#!/usr/bin/env python3
"""Forward ROS1 ActionCommand target yaw in robot frame to UDP."""

import argparse
import math
import os
import socket
import struct
import time

try:
    import rospy
    from dmsgs.msg import ActionCommand, VisionInfo
except ImportError as exc:
    raise SystemExit("Run this script in a ROS1 environment with dmsgs sourced.") from exc


PACKET = struct.Struct("<d?f")


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def default_robot_topic(prefix):
    robot_id = os.environ.get("ZJUDANCER_ROBOTID", "1")
    return f"/{prefix}_{robot_id}/" + ("ActionCommand" if prefix == "dbehavior" else "VisionInfo")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Forward ROS1 dbehavior target yaw as robot-frame yaw to UDP."
    )
    parser.add_argument(
        "--action-topic",
        default=default_robot_topic("dbehavior"),
        help="ROS1 dmsgs/ActionCommand topic to subscribe.",
    )
    parser.add_argument(
        "--vision-topic",
        default=default_robot_topic("dvision"),
        help="ROS1 dmsgs/VisionInfo topic used for robot global yaw.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15004, help="UDP destination port.")
    parser.add_argument("--rate", type=float, default=50.0, help="UDP send rate in Hz.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Seconds before action/vision input is treated as stale.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    state = {
        "target_yaw_global": None,
        "target_time": None,
        "robot_yaw_global": None,
        "robot_time": None,
    }

    rospy.init_node("ros1_goal_yaw_udp_sender", anonymous=True)
    rospy.loginfo(
        "Forwarding ROS1 target yaw %s + robot yaw %s to udp://%s:%d",
        args.action_topic,
        args.vision_topic,
        args.host,
        args.port,
    )

    def action_callback(msg):
        state["target_yaw_global"] = math.radians(float(msg.bodyCmd.t))
        state["target_time"] = time.time()

    def vision_callback(msg):
        state["robot_yaw_global"] = math.radians(float(msg.robot_pos.z))
        state["robot_time"] = time.time()

    def timer_callback(_event):
        now = time.time()
        valid = (
            state["target_yaw_global"] is not None
            and state["robot_yaw_global"] is not None
            and state["target_time"] is not None
            and state["robot_time"] is not None
            and now - state["target_time"] <= args.timeout
            and now - state["robot_time"] <= args.timeout
        )
        goal_yaw_b = 0.0
        if valid:
            goal_yaw_b = wrap_pi(state["target_yaw_global"] - state["robot_yaw_global"])
        sock.sendto(PACKET.pack(now, bool(valid), float(goal_yaw_b)), addr)

    rospy.Subscriber(args.action_topic, ActionCommand, action_callback, queue_size=10)
    rospy.Subscriber(args.vision_topic, VisionInfo, vision_callback, queue_size=10)
    rospy.Timer(rospy.Duration(1.0 / args.rate), timer_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
