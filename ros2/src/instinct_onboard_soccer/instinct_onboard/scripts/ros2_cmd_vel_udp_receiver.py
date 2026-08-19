#!/usr/bin/env python3
"""Receive UDP cmd_vel packets and publish them as ROS2 Twist messages."""

import argparse
import select
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


PACKET = struct.Struct("<dfff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive UDP velocity packets and publish ROS2 geometry_msgs/Twist."
    )
    parser.add_argument("--topic", default="/cmd_vel", help="ROS2 Twist topic to publish.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15000, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=50.0, help="ROS2 publish/check rate in Hz.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Seconds before stale UDP input is published as zero velocity.",
    )
    return parser.parse_args()


class CmdVelUdpReceiver(Node):
    def __init__(self, args):
        super().__init__("ros2_cmd_vel_udp_receiver")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_receive_time = None
        self.was_stale = True
        self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing UDP udp://{args.host}:{args.port} to ROS2 {args.topic}; "
            f"timeout={args.timeout:.2f}s"
        )

    def timer_callback(self):
        self._drain_udp()
        fresh = (
            self.last_receive_time is not None
            and time.time() - self.last_receive_time <= self.args.timeout
        )
        if fresh:
            self._publish_cmd(*self.last_cmd)
            if self.was_stale:
                self.get_logger().info("Received fresh UDP cmd_vel.")
                self.was_stale = False
        else:
            self._publish_cmd(0.0, 0.0, 0.0)
            if not self.was_stale:
                self.get_logger().warn("UDP cmd_vel timed out; publishing zero velocity.")
                self.was_stale = True

    def _drain_udp(self):
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            data, _ = self.sock.recvfrom(1024)
            if len(data) != PACKET.size:
                self.get_logger().warn(f"Ignoring malformed UDP packet: {len(data)} bytes")
                continue
            _, vx, vy, wz = PACKET.unpack(data)
            self.last_cmd = (float(vx), float(vy), float(wz))
            self.last_receive_time = time.time()

    def _publish_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.publisher.publish(msg)


def main():
    args = parse_args()
    rclpy.init()
    node = CmdVelUdpReceiver(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
