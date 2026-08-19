from __future__ import annotations

import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.hold_agent import PoseHoldAgent
from instinct_onboard.agents.tracking_agent import TrackerAgent
from instinct_onboard.ros_nodes.hightorque import HightorqueNode


class FallRecoveryAgent(TrackerAgent):
    """Tracker-style agent for the PiPlus H0W BeyondMimic fall-recovery policy."""

    def _get_position_ref_command_cmd_obs(self):
        return self._get_position_b_ref_command_cmd_obs()


class FallPostureDetector:
    """Consecutive-frame fall and face-up/face-down detector.

    The detector intentionally locks after selecting a posture. Recovery code should
    stop calling it after lock, but returning the locked posture here keeps the
    behavior stable if it is called again by mistake.
    """

    def __init__(
        self,
        fallen_abs_z_threshold: float,
        fallen_confirm_frames: int,
        face_axis: int,
        faceup_sign: float,
        face_threshold: float,
        posture_confirm_frames: int,
        classify_ang_vel_max: float,
    ):
        self.fallen_abs_z_threshold = float(fallen_abs_z_threshold)
        self.fallen_confirm_frames = max(1, int(fallen_confirm_frames))
        self.face_axis = int(face_axis)
        self.faceup_sign = float(faceup_sign)
        self.face_threshold = float(face_threshold)
        self.posture_confirm_frames = max(1, int(posture_confirm_frames))
        self.classify_ang_vel_max = float(classify_ang_vel_max)
        self.reset()

    def reset(self):
        self.fallen_count = 0
        self.faceup_count = 0
        self.facedown_count = 0
        self.fallen_confirmed = False
        self.locked_posture: str | None = None

    def update(self, projected_gravity: np.ndarray, angular_velocity: np.ndarray) -> str | None:
        if self.locked_posture is not None:
            return self.locked_posture

        gravity = np.asarray(projected_gravity, dtype=np.float32)
        ang_vel = np.asarray(angular_velocity, dtype=np.float32)

        if not self.fallen_confirmed:
            if abs(float(gravity[2])) < self.fallen_abs_z_threshold:
                self.fallen_count += 1
            else:
                self.fallen_count = 0

            if self.fallen_count >= self.fallen_confirm_frames:
                self.fallen_confirmed = True
                self.faceup_count = 0
                self.facedown_count = 0
            return None

        if float(np.linalg.norm(ang_vel)) >= self.classify_ang_vel_max:
            self.faceup_count = 0
            self.facedown_count = 0
            return None

        face_score = float(gravity[self.face_axis]) * self.faceup_sign
        if face_score > self.face_threshold:
            self.faceup_count += 1
            self.facedown_count = 0
        elif face_score < -self.face_threshold:
            self.facedown_count += 1
            self.faceup_count = 0
        else:
            self.faceup_count = 0
            self.facedown_count = 0

        if self.faceup_count >= self.posture_confirm_frames:
            self.locked_posture = "faceup"
        elif self.facedown_count >= self.posture_confirm_frames:
            self.locked_posture = "facedown"
        return self.locked_posture


class PiPlusH0WFallRecoveryNode(HightorqueNode):
    def __init__(
        self,
        *args,
        mode: str = "manual",
        motion_vis: bool = False,
        hold_final_pose: bool = False,
        fall_detector: FallPostureDetector | None = None,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.available_agents = {}
        self.current_agent_name: str | None = None
        self.mode = mode
        self.motion_vis = motion_vis
        self.hold_final_pose = hold_final_pose
        self.fall_detector = fall_detector
        self.locked_recovery_agent_name: str | None = None
        self.fall_recovery_final_frame_reached = False
        self._auto_fallen_logged = False

    def register_agent(self, name: str, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        self.reference_joint_state_publisher = self.create_publisher(JointState, "reference_joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.main_loop_timer = self.create_timer(0.02, self.main_loop_callback)
        if self.motion_vis:
            self.vis_timer = self.create_timer(0.1, self.vis_callback)

    def main_loop_callback(self):
        if self.mode == "auto":
            self._auto_main_loop_callback()
        else:
            self._manual_main_loop_callback()

    def _send_agent_action(self, agent):
        action, done = agent.step()
        self.send_action(
            action,
            agent.action_offset,
            agent.action_scale,
            agent.p_gains,
            agent.d_gains,
        )
        return done

    def _manual_main_loop_callback(self):
        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.get_logger().info("Press 'A' to match heading, then press 'L1' to start fall recovery.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        if self.joy_stick_data.A:
            self.get_logger().info(
                "A button pressed, matching fall-recovery motion to current heading.",
                throttle_duration_sec=2.0,
            )
            self.available_agents["fall_recovery"].match_to_current_heading()

        elif self.current_agent_name == "cold_start":
            agent = self.available_agents[self.current_agent_name]
            done = self._send_agent_action(agent)
            if done:
                self.get_logger().info("ColdStartAgent done, press 'L1' to start fall recovery.", throttle_duration_sec=10.0)
            if done and self.joy_stick_data.L1:
                self.get_logger().info("L1 button pressed, switching to fall-recovery agent.")
                self.current_agent_name = "fall_recovery"
                self.fall_recovery_final_frame_reached = False
                self.available_agents[self.current_agent_name].reset()

        elif self.current_agent_name == "fall_recovery":
            agent = self.available_agents[self.current_agent_name]
            done = self._send_agent_action(agent)
            if (
                done
                and not self.fall_recovery_final_frame_reached
                and agent.motion_cursor_idx >= agent.motion_data.total_num_frames - 1
            ):
                self.fall_recovery_final_frame_reached = True
                if self.hold_final_pose and "hold_final" in self.available_agents:
                    self.get_logger().info("FallRecoveryAgent done, switching to final-pose hold agent.")
                    self.current_agent_name = "hold_final"
                    self.available_agents[self.current_agent_name].reset()
                else:
                    self.get_logger().info("FallRecoveryAgent done, holding final frame.")

        elif self.current_agent_name == "hold_final":
            self._send_agent_action(self.available_agents[self.current_agent_name])

    def _auto_main_loop_callback(self):
        if self.current_agent_name is None:
            self.get_logger().info("Starting auto fall-recovery standby hold agent.")
            self.get_logger().info("Push the robot down; posture will be locked before recovery starts.")
            self.current_agent_name = "standby_hold"
            self.available_agents[self.current_agent_name].reset()
            if self.fall_detector is not None:
                self.fall_detector.reset()
            return

        if self.current_agent_name in ("standby_hold", "settle_hold"):
            if self.fall_detector is None:
                self.get_logger().error("Auto mode requires a FallPostureDetector.")
                return

            if self.fall_detector.fallen_confirmed and "settle_hold" in self.available_agents:
                self.current_agent_name = "settle_hold"
            self._send_agent_action(self.available_agents[self.current_agent_name])

            locked_posture = self.fall_detector.update(
                self._get_projected_gravity_obs(),
                self._get_imu_ang_vel_obs(),
            )
            if self.fall_detector.fallen_confirmed and not self._auto_fallen_logged:
                self.get_logger().warn("Fall detected; waiting for stable face-up/face-down confirmation.")
                self._auto_fallen_logged = True
            if locked_posture is not None:
                self.locked_recovery_agent_name = f"fall_recovery_{locked_posture}"
                self.get_logger().warn(
                    f"Locked {locked_posture} fall-recovery policy; recovery reference will not switch."
                )
                self.current_agent_name = self.locked_recovery_agent_name
                self.fall_recovery_final_frame_reached = False
                self.available_agents[self.current_agent_name].reset()
            return

        if self.current_agent_name in ("fall_recovery_faceup", "fall_recovery_facedown"):
            agent = self.available_agents[self.current_agent_name]
            done = self._send_agent_action(agent)
            if (
                done
                and not self.fall_recovery_final_frame_reached
                and agent.motion_cursor_idx >= agent.motion_data.total_num_frames - 1
            ):
                self.fall_recovery_final_frame_reached = True
                hold_name = self.current_agent_name.replace("fall_recovery", "hold_final")
                if self.hold_final_pose and hold_name in self.available_agents:
                    self.get_logger().info("FallRecoveryAgent done, switching to final-pose hold agent.")
                    self.current_agent_name = hold_name
                    self.available_agents[self.current_agent_name].reset()
                else:
                    self.get_logger().info("FallRecoveryAgent done, holding final frame.")
            return

        if self.current_agent_name in ("hold_final_faceup", "hold_final_facedown"):
            self._send_agent_action(self.available_agents[self.current_agent_name])

    def vis_callback(self):
        agent_name = self.locked_recovery_agent_name
        if agent_name is None:
            for candidate in ("fall_recovery", "fall_recovery_faceup", "fall_recovery_facedown"):
                if candidate in self.available_agents:
                    agent_name = candidate
                    break
        if agent_name is None:
            return
        agent: FallRecoveryAgent = self.available_agents[agent_name]
        cursor = agent.motion_cursor_idx

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.sim_joint_names
        js.position = agent.motion_data.joint_pos[cursor].tolist()
        js.velocity = agent.motion_data.joint_vel[cursor].tolist()
        js.effort = [0.0] * len(js.position)
        self.reference_joint_state_publisher.publish(js)

        pos = agent.motion_data.base_pos[cursor]
        quat = agent.motion_data.base_quat[cursor]
        transform = TransformStamped()
        transform.header.stamp = js.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "reference_base_link"
        transform.transform.translation.x = float(pos[0])
        transform.transform.translation.y = float(pos[1])
        transform.transform.translation.z = float(pos[2])
        transform.transform.rotation.w = float(quat[0])
        transform.transform.rotation.x = float(quat[1])
        transform.transform.rotation.y = float(quat[2])
        transform.transform.rotation.z = float(quat[3])
        self.tf_broadcaster.sendTransform(transform)


def _make_pose_hold_agent(source_agent, target_joint_pos: np.ndarray, kpkd_factor: float) -> PoseHoldAgent:
    return PoseHoldAgent(
        target_joint_pos=target_joint_pos,
        action_scale=source_agent.action_scale,
        action_offset=source_agent.action_offset,
        p_gains=source_agent.p_gains * kpkd_factor,
        d_gains=source_agent.d_gains * kpkd_factor,
    )


def _require_args(args, names: tuple[str, ...]):
    missing = [name for name in names if getattr(args, name) is None]
    if missing:
        raise ValueError(f"Missing required arguments for {args.mode} mode: {', '.join('--' + name for name in missing)}")


def main(args):
    if args.mode == "manual":
        _require_args(args, ("logdir", "motion_dir"))
    else:
        _require_args(args, ("faceup_logdir", "faceup_motion_dir", "facedown_logdir", "facedown_motion_dir"))

    rclpy.init()

    fall_detector = None
    if args.mode == "auto":
        fall_detector = FallPostureDetector(
            fallen_abs_z_threshold=args.fallen_abs_z_threshold,
            fallen_confirm_frames=args.fallen_confirm_frames,
            face_axis=args.face_axis,
            faceup_sign=args.faceup_sign,
            face_threshold=args.face_threshold,
            posture_confirm_frames=args.posture_confirm_frames,
            classify_ang_vel_max=args.classify_ang_vel_max,
        )

    node = PiPlusH0WFallRecoveryNode(
        node_name="piplus_h0w_fall_recovery_node",
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        mode=args.mode,
        dryrun=not args.nodryrun,
        joint_pos_protect_ratio=args.joint_pos_protect_ratio,
        kp_factor=args.kp_factor,
        kd_factor=args.kd_factor,
        torque_limits_ratio=args.torque_limits_ratio,
        motion_vis=args.motion_vis,
        hold_final_pose=args.hold_final_pose,
        fall_detector=fall_detector,
    )

    if args.mode == "manual":
        recovery_agent = FallRecoveryAgent(
            logdir=args.logdir,
            motion_file_dir=args.motion_dir,
            ros_node=node,
        )
        cold_start_agent = recovery_agent.get_cold_start_agent(args.startup_step_size, args.kpkd_factor)
        hold_final_agent = _make_pose_hold_agent(
            recovery_agent,
            recovery_agent.motion_data.joint_pos[-1],
            args.hold_kpkd_factor,
        )

        node.register_agent("cold_start", cold_start_agent)
        node.register_agent("fall_recovery", recovery_agent)
        node.register_agent("hold_final", hold_final_agent)
    else:
        faceup_agent = FallRecoveryAgent(
            logdir=args.faceup_logdir,
            motion_file_dir=args.faceup_motion_dir,
            ros_node=node,
        )
        facedown_agent = FallRecoveryAgent(
            logdir=args.facedown_logdir,
            motion_file_dir=args.facedown_motion_dir,
            ros_node=node,
        )

        node.register_agent("fall_recovery_faceup", faceup_agent)
        node.register_agent("fall_recovery_facedown", facedown_agent)
        node.register_agent(
            "hold_final_faceup",
            _make_pose_hold_agent(faceup_agent, faceup_agent.motion_data.joint_pos[-1], args.hold_kpkd_factor),
        )
        node.register_agent(
            "hold_final_facedown",
            _make_pose_hold_agent(facedown_agent, facedown_agent.motion_data.joint_pos[-1], args.hold_kpkd_factor),
        )

    node.start_ros_handlers()
    if args.mode == "auto":
        standby_joint_pos = node._get_joint_pos_obs().copy()
        reference_agent = node.available_agents["fall_recovery_faceup"]
        node.register_agent(
            "standby_hold",
            _make_pose_hold_agent(reference_agent, standby_joint_pos, args.standby_kpkd_factor),
        )
        node.register_agent(
            "settle_hold",
            _make_pose_hold_agent(reference_agent, standby_joint_pos, args.settle_kpkd_factor),
        )
    node.get_logger().info("PiPlusH0WFallRecoveryNode is ready to run.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    finally:
        if getattr(node, "control_uuid", None) is not None:
            node._turn_off_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="PiPlus H0W Fall Recovery Node")
    parser.add_argument("--mode", choices=("manual", "auto"), default="manual", help="Manual L1 or automatic fall-triggered recovery.")
    parser.add_argument("--logdir", type=str, help="Directory containing exported fall-recovery policy for manual mode.")
    parser.add_argument("--motion_dir", type=str, help="Directory containing fall-recovery motion .npz files for manual mode.")
    parser.add_argument("--faceup_logdir", type=str, help="Directory containing exported face-up recovery policy.")
    parser.add_argument("--faceup_motion_dir", type=str, help="Directory containing face-up recovery motion .npz files.")
    parser.add_argument("--facedown_logdir", type=str, help="Directory containing exported face-down recovery policy.")
    parser.add_argument("--facedown_motion_dir", type=str, help="Directory containing face-down recovery motion .npz files.")
    parser.add_argument("--startup_step_size", type=float, default=0.05, help="Cold start joint step size.")
    parser.add_argument("--kpkd_factor", type=float, default=0.5, help="Cold start gain multiplier.")
    parser.add_argument("--hold_kpkd_factor", type=float, default=0.5, help="Final pose hold gain multiplier.")
    parser.add_argument("--standby_kpkd_factor", type=float, default=0.3, help="Auto standby pose hold gain multiplier.")
    parser.add_argument("--settle_kpkd_factor", type=float, default=0.1, help="Auto post-fall settle pose hold gain multiplier.")
    parser.add_argument("--kp_factor", type=float, default=1.0, help="Runtime KP multiplier.")
    parser.add_argument("--kd_factor", type=float, default=1.0, help="Runtime KD multiplier.")
    parser.add_argument("--torque_limits_ratio", type=float, default=1.0, help="Torque limit multiplier.")
    parser.add_argument("--joint_pos_protect_ratio", type=float, default=2.0, help="Joint position safety ratio.")
    parser.add_argument("--fallen_abs_z_threshold", type=float, default=0.55, help="Projected-gravity |z| threshold for fallen detection.")
    parser.add_argument("--fallen_confirm_frames", type=int, default=15, help="Consecutive fallen frames before posture classification.")
    parser.add_argument("--face_axis", type=int, default=0, help="Projected-gravity axis used for face-up/face-down classification.")
    parser.add_argument("--faceup_sign", type=float, default=-1.0, help="Sign that makes face-up projected-gravity score positive.")
    parser.add_argument("--face_threshold", type=float, default=0.65, help="Projected-gravity face score threshold.")
    parser.add_argument("--posture_confirm_frames", type=int, default=10, help="Consecutive stable posture frames before locking recovery.")
    parser.add_argument("--classify_ang_vel_max", type=float, default=2.0, help="Maximum IMU angular-velocity norm for posture-frame accumulation.")
    parser.add_argument("--motion_vis", action="store_true", default=False, help="Publish reference motion for RViz.")
    parser.add_argument("--hold_final_pose", action="store_true", default=False, help="Switch to final-pose hold after recovery instead of continuing the recovery policy at the final reference frame.")
    parser.add_argument("--nodryrun", action="store_true", default=False, help="Send commands to the real robot.")
    parser.add_argument("--debug", action="store_true", default=False, help="Enable debugpy attach.")

    args = parser.parse_args()

    if args.debug:
        import debugpy

        ip_address = ("0.0.0.0", 6789)
        print("Process: " + " ".join(sys.argv[:]))
        print("Is waiting for attach at address: %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)
