#!/usr/bin/env python3
"""Receive UDP target yaw packets and publish ROS2 Float32."""

import argparse
import select
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


PACKET = struct.Struct("<d?f")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive UDP robot-frame target yaw and publish ROS2 std_msgs/Float32."
    )
    parser.add_argument(
        "--topic",
        default="/ros1_goal_yaw_b",
        help="ROS2 Float32 topic to publish. Unit: radians in robot/body frame.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15004, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish/check rate in Hz.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Seconds before stale target yaw input is ignored.",
    )
    return parser.parse_args()


class GoalYawUdpReceiver(Node):
    def __init__(self, args):
        super().__init__("ros2_goal_yaw_udp_receiver")
        self.args = args
        self.publisher = self.create_publisher(Float32, args.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.last_goal_yaw_b = None
        self.last_receive_time = None
        self.was_valid = False
        self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing UDP udp://{args.host}:{args.port} target yaw to ROS2 {args.topic}; "
            f"timeout={args.timeout:.2f}s"
        )

    def timer_callback(self):
        self._drain_udp()
        fresh = (
            self.last_goal_yaw_b is not None
            and self.last_receive_time is not None
            and time.time() - self.last_receive_time <= self.args.timeout
        )
        if not fresh:
            if self.was_valid:
                self.get_logger().warn("UDP target yaw timed out; publishing nothing.")
                self.was_valid = False
            return

        msg = Float32()
        msg.data = float(self.last_goal_yaw_b)
        self.publisher.publish(msg)
        if not self.was_valid:
            self.get_logger().info("Received fresh ROS1 target yaw.")
            self.was_valid = True

    def _drain_udp(self):
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            data, _ = self.sock.recvfrom(1024)
            if len(data) != PACKET.size:
                self.get_logger().warn(f"Ignoring malformed UDP packet: {len(data)} bytes")
                continue
            _, valid, goal_yaw_b = PACKET.unpack(data)
            if not valid:
                self.last_goal_yaw_b = None
                self.last_receive_time = None
                continue
            self.last_goal_yaw_b = float(goal_yaw_b)
            self.last_receive_time = time.time()


def main():
    args = parse_args()
    rclpy.init()
    node = GoalYawUdpReceiver(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
