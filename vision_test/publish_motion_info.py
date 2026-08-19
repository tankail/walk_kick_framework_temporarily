#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单独视觉测试用的轻量 /dmotion_X/MotionInfo 发布节点。

完整框架里 /dmotion_1/MotionInfo 由 dancer-io 发布。单独测试不跑 dancer-io,
本节点只把 ROS1 head MotorState 反馈(来自 ros1_head_state_udp_receiver) 组装成
MotionInfo.curPlat.yaw/pitch 发布出去, 让 dvision 和 overlay 都能拿到真实头部角度。

链路: ROS2 bringup -> /joint_states -> ros2_head_state_udp_sender -> UDP 15002
      -> ros1_head_state_udp_receiver -> /livelybot_real_real/head_*_controller/state
      -> 本节点 -> /dmotion_X/MotionInfo -> dvision / overlay

依赖: rospy, dmsgs(MotionInfo), livelybot_msg(MotorState)。用系统 python3 跑。
"""

import argparse
import math

try:
    import rospy
    from dmsgs.msg import MotionInfo, HeadCommand
    from geometry_msgs.msg import Vector3
    try:
        from livelybot_msg.msg import MotorState
    except ImportError:
        from livelybot_serial.msg import MotorState
except ImportError as exc:
    raise SystemExit(
        "Run this script in a ROS1 environment with dmsgs + livelybot_msg sourced."
    ) from exc


def main():
    p = argparse.ArgumentParser(description="Publish /dmotion_X/MotionInfo head angles")
    p.add_argument("--robot-id", type=int, default=1)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--yaw-topic",
                   default="/livelybot_real_real/head_yaw_joint_controller/state")
    p.add_argument("--pitch-topic",
                   default="/livelybot_real_real/head_pitch_joint_controller/state")
    args = p.parse_args()

    rospy.init_node("publish_motion_info", anonymous=True)
    topic = "/dmotion_{}/MotionInfo".format(int(args.robot_id))
    pub = rospy.Publisher(topic, MotionInfo, queue_size=10)

    state = {"yaw": 0.0, "pitch": 0.0, "yaw_ok": False, "pitch_ok": False}

    def yaw_cb(msg):
        state["yaw"] = float(msg.pos)
        state["yaw_ok"] = True

    def pitch_cb(msg):
        state["pitch"] = float(msg.pos)
        state["pitch_ok"] = True

    rospy.Subscriber(args.yaw_topic, MotorState, yaw_cb, queue_size=10)
    rospy.Subscriber(args.pitch_topic, MotorState, pitch_cb, queue_size=10)

    rospy.loginfo("publishing %s from %s / %s", topic, args.yaw_topic, args.pitch_topic)
    print("publishing {} head angles -> {}".format(
        "real" if (state["yaw_ok"] or state["pitch_ok"]) else "(waiting feedback)",
        topic), flush=True)

    rate = rospy.Rate(max(1.0, float(args.rate)))
    while not rospy.is_shutdown():
        m = MotionInfo()
        m.stable = True
        m.forward_or_backward = False
        m.lower_board_connected = True
        m.support_stablizer_flag = True
        m.curPlat = HeadCommand()
        # MotorState.pos 是弧度, 必须转度 (dancer-io 也是 rad2deg 后发 curPlat)
        m.curPlat.yaw = math.degrees(state["yaw"]) if state["yaw_ok"] else 0.0
        m.curPlat.pitch = math.degrees(state["pitch"]) if state["pitch_ok"] else 0.0
        m.odometry = Vector3()
        m.imuRPY = Vector3()
        pub.publish(m)
        rate.sleep()


if __name__ == "__main__":
    main()
