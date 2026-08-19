#!/usr/bin/env python3
"""Forward ROS2 head joint feedback to UDP for ROS1 MotionInfo feedback."""

import argparse
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PACKET = struct.Struct("<dffff")


def parse_args():
    parser = argparse.ArgumentParser(description="Forward ROS2 head JointState feedback to UDP.")
    parser.add_argument("--topic", default="/joint_states", help="ROS2 JointState topic to subscribe.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15002, help="UDP destination port.")
    parser.add_argument("--yaw-name", default="head_yaw_joint", help="Head yaw joint name.")
    parser.add_argument("--pitch-name", default="head_pitch_joint", help="Head pitch joint name.")
    parser.add_argument("--yaw-index", type=int, default=20, help="Fallback yaw index when JointState names are empty.")
    parser.add_argument("--pitch-index", type=int, default=21, help="Fallback pitch index when JointState names are empty.")
    parser.add_argument(
        "--max-abs-position",
        type=float,
        default=3.2,
        help="Ignore head positions whose absolute value exceeds this many radians.",
    )
    return parser.parse_args()


class HeadStateUdpSender(Node):
    def __init__(self, args):
        super().__init__("ros2_head_state_udp_sender")
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (args.host, args.port)
        self.subscription = self.create_subscription(JointState, args.topic, self.callback, 10)
        self.get_logger().info(
            f"Forwarding ROS2 {args.topic} head feedback to udp://{args.host}:{args.port}"
        )

    def callback(self, msg):
        yaw = self._get_value(msg.name, msg.position, self.args.yaw_name, self.args.yaw_index)
        pitch = self._get_value(msg.name, msg.position, self.args.pitch_name, self.args.pitch_index)
        if yaw is None or pitch is None:
            self.get_logger().warn(
                "JointState does not contain head yaw/pitch feedback.",
                throttle_duration_sec=2.0,
            )
            return
        yaw_vel = self._get_value(msg.name, msg.velocity, self.args.yaw_name, self.args.yaw_index, default=0.0)
        pitch_vel = self._get_value(msg.name, msg.velocity, self.args.pitch_name, self.args.pitch_index, default=0.0)
        if abs(yaw) > self.args.max_abs_position or abs(pitch) > self.args.max_abs_position:
            self.get_logger().warn(
                f"Ignoring implausible head state yaw={yaw:.3f}, pitch={pitch:.3f}",
                throttle_duration_sec=2.0,
            )
            return
        packet = PACKET.pack(
            time.time(),
            yaw,
            pitch,
            float(yaw_vel),
            float(pitch_vel),
        )
        self.sock.sendto(packet, self.addr)

    @staticmethod
    def _get_value(names, values, name, fallback_index, default=None):
        if not values:
            return default
        if names and name in names:
            index = names.index(name)
            if index < len(values):
                return float(values[index])
        if fallback_index >= 0 and fallback_index < len(values):
            return float(values[fallback_index])
        return default


def main():
    args = parse_args()
    rclpy.init()
    node = HeadStateUdpSender(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
