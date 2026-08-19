#!/usr/bin/env python3
"""Receive UDP head joint targets and publish them as ROS2 JointState."""

import argparse
import select
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PACKET = struct.Struct("<dff")


def parse_args():
    parser = argparse.ArgumentParser(description="Receive UDP head targets and publish ROS2 JointState.")
    parser.add_argument("--topic", default="/pi_plus_absolute", help="ROS2 JointState topic to publish.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15001, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=50.0, help="ROS2 publish/check rate in Hz.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Seconds before stale UDP input is ignored.")
    return parser.parse_args()


class HeadJointUdpReceiver(Node):
    def __init__(self, args):
        super().__init__("ros2_head_joint_udp_receiver")
        self.args = args
        self.publisher = self.create_publisher(JointState, args.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.last_cmd = None
        self.last_receive_time = None
        self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing UDP udp://{args.host}:{args.port} to ROS2 {args.topic}; "
            f"timeout={args.timeout:.2f}s"
        )

    def timer_callback(self):
        self._drain_udp()
        if self.last_cmd is None or time.time() - self.last_receive_time > self.args.timeout:
            return
        yaw, pitch = self.last_cmd
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["head_yaw_joint", "head_pitch_joint"]
        msg.position = [yaw, pitch]
        self.publisher.publish(msg)

    def _drain_udp(self):
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            data, _ = self.sock.recvfrom(1024)
            if len(data) != PACKET.size:
                self.get_logger().warn(f"Ignoring malformed UDP packet: {len(data)} bytes")
                continue
            _, yaw, pitch = PACKET.unpack(data)
            self.last_cmd = (float(yaw), float(pitch))
            self.last_receive_time = time.time()


def main():
    args = parse_args()
    rclpy.init()
    node = HeadJointUdpReceiver(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
