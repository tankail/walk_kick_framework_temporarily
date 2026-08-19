from dbehavior.core import Skill
from dbehavior.btree.leaf import BallSeen, Wait
from dbehavior.btree.core import Status
from dbehavior.btree.branch import Selector, Sequence, DynamicGuardSelector, Parallel
from dbehavior.util import Timer, VecPos
from dbehavior.util.mathutil import (
    abs_angle_diff, angle_normalization, get_angle, get_magnitude)
from math import fabs
import rospy
import time


class TurnAround(Skill):
    """
    Turn Around.
    """

    def __init__(self, bb, look_down=True, follow_ball=True, target_angle=150):
        super(TurnAround, self).__init__(bb)
        self.started_angle = 0
        self.look_down = look_down
        self.follow_ball = follow_ball

        self.TARGET_ANGLE_DELTA = target_angle

    def on_start(self):
        self.started_angle = self.bb.vision_info.robot_pos.z
        self.target_angle = self.started_angle + self.TARGET_ANGLE_DELTA

    def reset(self):
        super(TurnAround, self).reset()
        self.started_angle = 0

    def execute(self):
        self.look_at(15, 0)
        if self._turned_to_target(self.target_angle):
            self.crouch()
            return Status.SUCCEEDED
        else:
            # x = self.bb.vision_info.robot_pos.x
            # y = self.bb.vision_info.robot_pos.y
            # yaw = self.bb.vision_info.robot_pos.z + 180
            # self.bb.action_cmd.bodyCmd = self.action_generator.walk_pos(
            #     x, y, yaw)
            self.bb.action_cmd.bodyCmd = self.action_generator.turn(
                self.target_angle)
            return Status.RUNNING


class TurnTo(Skill):

    def __init__(self, bb, target_angle):
        super(TurnTo, self).__init__(bb)

        self.target_angle = target_angle
        self.angle_tol = 10
        self.angel_per_turn = 8

    def execute(self):
        if abs_angle_diff(self.bb.vision_info.robot_pos.z -
                          self.target_angle) <= self.angle_tol:
            return Status.SUCCEEDED
        else:
            angle = angle_normalization(self.bb.vision_info.robot_pos.z -
                                        self.target_angle)

            if angle > 0:
                self.turn(self.angel_per_turn)
                # self.walk(-1, 0, self.angel_per_turn)
            else:
                self.turn(-self.angel_per_turn)
                # self.walk(-1, 0, -self.angel_per_turn)
            return Status.RUNNING


class ScanField(Skill):
    """
    Scan field by rotating head.
    """

    def __init__(self, bb, timeout=1):
        """
        Initialize ScanField.

        :param bb: blackboard from behavior tree
        :type bb: DBlackboard
        :param timeout: timeout for timer (used for debug)
        :type timeout: float, int
        """
        super(ScanField, self).__init__(bb)
        self.timeout = timeout
        self.timer = Timer(timeout)
        self.gaze_plats = [VecPos(15, 75), VecPos(15, 50), VecPos(15, 25), VecPos(15, 0), VecPos(15, -25), VecPos(15, -50), VecPos(15, -75)]
        self.iter = iter(self.gaze_plats)
        self.cur_plat = next(self.iter)
        self.keep = False
        self.pitch_speed = 10
        self.scan_step_deg = 2.5

    def reset(self):
        super(ScanField, self).reset()
        self.timer = Timer(time_target=self.timeout)
        self.iter = iter(self.gaze_plats)
        print("reset gaze_plats:")
        self.cur_plat = next(self.iter)
        self.keep = False

    def on_start(self):
        super(ScanField, self).on_start()
        if self.head_got_dest(self.cur_plat.x, self.cur_plat.y):
            self.cur_plat = next(self.iter)

    def on_end(self):
        if self.gaze_plats[0].y - self.gaze_plats[-1].y > 30:
            self.gaze_plats.reverse()
        self.iter = iter(self.gaze_plats)
        self.cur_plat = next(self.iter)
        self.keep = False

    def execute(self):
        for pos in self.gaze_plats:
            print(f"({pos.x}, {pos.y})")
        if self._status == Status.FRESH:
            self.timer.restart()
        self.look_at_limited(self.cur_plat.x, self.cur_plat.y)
        if self.head_got_dest(self.cur_plat.x, self.cur_plat.y):
            print('got dest', self.cur_plat.x, self.cur_plat.y)
            if not self.keep:
                self.keep = True
                self.timer.restart()
            elif self.timer.finished():
                self.keep = False
                try:
                    self.cur_plat = next(self.iter)
                except StopIteration:
                    return Status.SUCCEEDED
        return Status.RUNNING

    def look_at_limited(self, pitch, yaw):
        cur_plat = self.bb.motion_info.curPlat

        def limit_step(current, target):
            diff = target - current
            if fabs(diff) <= self.scan_step_deg:
                return target
            return current + self.scan_step_deg if diff > 0 else current - self.scan_step_deg

        cmd_pitch = limit_step(cur_plat.pitch, pitch)
        cmd_yaw = limit_step(cur_plat.yaw, yaw)
        self.look_at(cmd_pitch, cmd_yaw, self.pitch_speed, self.pitch_speed)


class ScanFieldSlow(ScanField):

    def __init__(self, bb, timeout=1):
        super(ScanFieldSlow, self).__init__(bb, timeout)
        self.pitch_speed = 0.5


# yapf: disable
class EntryScanField(ScanField):

    def __init__(self, bb, timeout=1):
        super(EntryScanField, self).__init__(bb, timeout=0.7)
        self.gaze_plats = [
            VecPos(15, 60), VecPos(15, 30),
            VecPos(15, 0), VecPos(15, -30), VecPos(15, -60)]


class ScanFieldNew(ScanField):

    def __init__(self, bb, timeout=0.3):
        super(ScanFieldNew, self).__init__(bb, timeout)
        self.gaze_plats = [
            VecPos(15, 75),
            VecPos(15, 60),
            VecPos(15, 30),
            VecPos(15, 0),
            VecPos(15, -30),
            VecPos(15, -60),
            VecPos(15, -75)
        ]


class ScanFieldDown(ScanField):

    def __init__(self, bb, timeout=0.2):
        super(ScanFieldDown, self).__init__(bb, timeout)
        # 精简注视点 + 加大单步限速 + 缩短停留, 让"完整扫头"从 25-45s 缩短到 ~10s,
        # 这样"扫完头 -> 转身 -> 再扫头"的循环不会太慢(用户要求先扫完再转身)。
        self.gaze_plats = [
            VecPos(10, 75), VecPos(10, 0), VecPos(10, -75),
            VecPos(33, -75), VecPos(33, 0), VecPos(33, 75)]
        self.pitch_speed = 3.0
        self.scan_step_deg = 10.0
# yapf: enable

"""

class TrackBall(Skill):
    
    #Track ball by rotating head and centering the ball in FOV.
    

    def __init__(self, bb, speed=1,scan_timeout = 3.5, timeout = 0.8 , times = 3):
        super(TrackBall, self).__init__(bb)
        self.times = times
        self.now_times = 0
        self.TRACK_THRESH = 3
        self.speed = speed
        self.scan_timeout = scan_timeout
        self.scan_timer = Timer(self.scan_timeout)
        self.scan =  False
        self.gaze_plats = [VecPos(15, 90), VecPos(15, 0), VecPos(15, -90)]
        self.iter = iter(self.gaze_plats)
        self.cur_plat = next(self.iter)
        #for scan
        self.timeout = timeout
        self.timer = Timer(timeout)

        self.keep = False
        self.pitch_speed = 10
        self.scan_status = False
        
    def on_start(self):
        super(TrackBall, self).on_start()
        if self.head_got_dest(self.cur_plat.x, self.cur_plat.y):
            self.cur_plat = next(self.iter)
        

    def on_end(self):
        if self.gaze_plats[0].y - self.gaze_plats[-1].y > 30:
            self.gaze_plats.reverse()
        self.iter = iter(self.gaze_plats)
        self.cur_plat = next(self.iter)
        self.keep = False


    def execute(self):
        if self.scan_timer.finished():
            self.now_times += 1
            if self.now_times == self.times:
                self.scan = False
                self.bb.continue_ball_seen = True
            else:
                self.scan =  True
            
            
        if self.scan:
            if self.scan_status == False:
                self.scan_status = True
                self.timer.restart()
            self.look_at(self.cur_plat.x, self.cur_plat.y, self.pitch_speed,
                        self.pitch_speed)
            if self.head_got_dest(self.cur_plat.x, self.cur_plat.y):
                if not self.keep:
                    self.keep = True
                    self.timer.restart()
                elif self.timer.finished():
                    self.keep = False
                    try:
                        self.cur_plat = next(self.iter)
                    except StopIteration:
                        self.scan = False
                        self.scan_timer.restart()
                        #return Status.SUCCEEDED
            
            return Status.RUNNING

        
        ball = self.bb.vision_info.ball_field
        if ball.x > 10 and ball.x < 50 and fabs(ball.y) < 30:
            self.look_at(45, 0)
            self.scan = False
            self.scan_timer = Timer(self.scan_timeout)
        else:
            track = self.bb.vision_info.ballTrack
            cur_plat = self.bb.motion_info.curPlat

            pitch = track.pitch \
                if fabs(track.pitch - cur_plat.pitch) > self.TRACK_THRESH else cur_plat.pitch
            yaw = track.yaw \
                if fabs(track.yaw - cur_plat.yaw) > self.TRACK_THRESH else cur_plat.yaw
            #if self.scan == False:
                
            self.look_at(pitch, yaw, self.speed, self.speed)
            #self.bb.last_look = [pitch, yaw, self.speed, self.speed]
        return Status.SUCCEEDED

""" 
class TrackBall(Skill):


    def __init__(self, bb, speed=2.0):
        super(TrackBall, self).__init__(bb)
        self.TRACK_THRESH = 2.0
        self.speed = speed
        self.track_gain = 0.65
        self.pitch_min = -10
        self.pitch_max = 45
        self.yaw_min = -75
        self.yaw_max = 75
        self.pitch_step_deg = 3.0
        self.yaw_step_deg = 5.0
        self.last_vision_seq = None
        self.last_target_pitch = None
        self.last_target_yaw = None
        self.last_run_time = None

    def _limit_step(self, current, target, max_step):
        diff = target - current
        if fabs(diff) <= max_step:
            return target
        return current + max_step if diff > 0 else current - max_step

    def on_start(self):
        super(TrackBall, self).on_start()
        now = time.time()
        if self.last_run_time is None or now - self.last_run_time > 0.4:
            self.last_vision_seq = None
            self.last_target_pitch = None
            self.last_target_yaw = None

    def execute(self):
        track = self.bb.vision_info.ballTrack
        cur_plat = self.bb.motion_info.curPlat

        pitch_error = track.pitch
        yaw_error = track.yaw

        pitch = self.last_target_pitch \
            if self.last_target_pitch is not None else cur_plat.pitch
        yaw = self.last_target_yaw \
            if self.last_target_yaw is not None else cur_plat.yaw

        vision_seq = getattr(self.bb, 'vision_info_seq', None)
        has_new_frame = vision_seq is None or vision_seq != self.last_vision_seq
        if has_new_frame:
            target_pitch = pitch
            target_yaw = yaw
            if fabs(pitch_error) > self.TRACK_THRESH:
                target_pitch = cur_plat.pitch - pitch_error * self.track_gain
            if fabs(yaw_error) > self.TRACK_THRESH:
                target_yaw = cur_plat.yaw + yaw_error * self.track_gain
            pitch = self._limit_step(pitch, target_pitch, self.pitch_step_deg)
            yaw = self._limit_step(yaw, target_yaw, self.yaw_step_deg)
            self.last_vision_seq = vision_seq

        yaw = max(self.yaw_min, min(self.yaw_max, yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, pitch))
        self.last_target_pitch = pitch
        self.last_target_yaw = yaw
        self.last_run_time = time.time()

        self.look_at(pitch, yaw, self.speed, self.speed)
        return Status.SUCCEEDED



class GoToLastBall(Skill):
    """
    Go to the last ball global position in memory
    """

    def __init__(self, bb):
        super(GoToLastBall, self).__init__(bb)
        self.TRACK_THRESH = 3

    def execute(self):
        if not self.bb.ball_global_last:
            self.crouch()
            self.look_at(15, 0)
            rospy.warn('Lost ball_global_last!')
            return Status.FAILED

        pos = self.bb.ball_global_last
        self.final_dest = pos
        self.dest = pos
        if self._got_dest_simple(pos, dis_tol=50, angle_tol=15):
            self.crouch()
            return Status.SUCCEEDED
        else:
            self.goto_global(pos)
            return Status.RUNNING
        return Status.SUCCEEDED


class TurnToLastBall(Skill):
    """
    After a short visual dropout, turn toward the last base_link ball bearing
    before falling back to a full field scan.
    """

    def __init__(self, bb, duration=0.7, recent_timeout=1.2):
        super(TurnToLastBall, self).__init__(bb)
        self.duration = duration
        self.recent_timeout = recent_timeout
        self.timer = Timer(duration)
        self.valid = False
        self.target_angle = 0
        self.head_pitch = 15
        self.head_yaw = 0

    def on_start(self):
        super(TurnToLastBall, self).on_start()
        self.timer = Timer(self.duration)
        self.valid = False
        if self.bb.ball_field_last is None:
            return
        if self.bb.timer_ball_lost.elapsed() > self.recent_timeout:
            return

        ball = self.bb.ball_field_last
        ball_bearing = get_angle(ball)
        self.target_angle = self.bb.vision_info.robot_pos.z + ball_bearing
        self.head_yaw = max(-75, min(75, ball_bearing))
        self.head_pitch = 33 if get_magnitude(ball) < 1.0 else 15
        self.valid = True

    def execute(self):
        if not self.valid:
            return Status.SUCCEEDED
        self.look_at(self.head_pitch, self.head_yaw)
        if self.timer.finished():
            self.crouch()
            return Status.SUCCEEDED
        self.bb.action_cmd.bodyCmd = self.action_generator.turn(
            self.target_angle)
        return Status.RUNNING


class BodyTurnStep(Skill):
    """
    Turn body for a short fixed time.

    Route B VisionInfo.robot_pos.z is not reliable enough to decide whether a
    turn reached its target, so search uses time-bounded body turns between
    head-only scans.
    """

    def __init__(self, bb, turn_angle=135, duration=3.5):
        super(BodyTurnStep, self).__init__(bb)
        self.turn_angle = turn_angle
        self.duration = duration
        self.timer = Timer(duration)

    def on_start(self):
        super(BodyTurnStep, self).on_start()
        self.timer = Timer(self.duration)
        self.target_angle = self.bb.vision_info.robot_pos.z + self.turn_angle

    def execute(self):
        self.look_at(15, 0)
        if self.timer.finished():
            self.crouch()
            return Status.SUCCEEDED
        self.bb.action_cmd.bodyCmd = self.action_generator.turn(
            self.target_angle)
        return Status.RUNNING


class HeadScanThenBodyTurn(Sequence):
    """
    Head scan first. If still no ball, turn body a little and scan again.
    """

    def __init__(self, bb):
        super(HeadScanThenBodyTurn, self).__init__(
            TurnToLastBall(bb), ScanFieldDown(bb), BodyTurnStep(bb))


class SeekBall(DynamicGuardSelector):
    """
    Track ball if seen, otherwise repeat head scan and short body turn.
    """

    def __init__(self, bb):
        children = [(TrackBall(bb), BallSeen()),
                    (HeadScanThenBodyTurn(bb), None)]
        super(SeekBall, self).__init__(children)

    def run(self):
        self.get_bb().set_state('searching')
        super(SeekBall, self).run()


class FindBall(DynamicGuardSelector):
    """
    Track ball if see ball, or scan field without turn around
    """

    def __init__(self, bb, look_down=False):
        if look_down:
            children = [(TrackBall(bb), BallSeen()), (ScanFieldDown(bb), None)]
        else:
            children = [(TrackBall(bb), BallSeen()), (ScanFieldNew(bb), None)]
        super(FindBall, self).__init__(children)

    def run(self):
        self.get_bb().set_state('searching')
        super(FindBall, self).run()


class FindLastBall(Sequence):
    """
    Go to last seen ball position and scan for ball
    """

    def __init__(self, bb):
        args = (Parallel(ScanFieldDown(bb),
                         GoToLastBall(bb)), ScanFieldDown(bb),
                TurnAround(bb, target_angle=180), ScanFieldDown(bb))
        super(FindLastBall, self).__init__(*args)

    def run(self):
        self.get_bb().set_state('searching')
        super(FindLastBall, self).run()
