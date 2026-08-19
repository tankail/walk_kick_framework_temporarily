#!/usr/bin/env python3
"""Receive UDP IMU packets and publish as ROS2 sensor_msgs/Imu on /imu/data."""

import argparse
import select
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


PACKET = struct.Struct("<d10f")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive UDP IMU and publish ROS2 sensor_msgs/Imu."
    )
    parser.add_argument("--topic", default="/imu/data", help="ROS2 Imu topic to publish.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15008, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=100.0, help="Publish/check rate in Hz.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Seconds before stale IMU is ignored.")
    return parser.parse_args()


class ImuUdpReceiver(Node):
    def __init__(self, args):
        super().__init__("ros2_imu_udp_receiver")
        self.args = args
        self.publisher = self.create_publisher(Imu, args.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.last_imu = None
        self.last_receive_time = None
        self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing UDP udp://{args.host}:{args.port} IMU to ROS2 {args.topic}; "
            f"timeout={args.timeout:.2f}s"
        )

    def timer_callback(self):
        self._drain_udp()
        if self.last_imu is None or self.last_receive_time is None:
            return
        if time.time() - self.last_receive_time > self.args.timeout:
            return
        _t, qw, qx, qy, qz, avx, avy, avz, lax, lay, laz = self.last_imu
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.angular_velocity.x = avx
        msg.angular_velocity.y = avy
        msg.angular_velocity.z = avz
        msg.linear_acceleration.x = lax
        msg.linear_acceleration.y = lay
        msg.linear_acceleration.z = laz
        self.publisher.publish(msg)

    def _drain_udp(self):
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            data, _ = self.sock.recvfrom(256)
            if len(data) != PACKET.size:
                self.get_logger().warn(f"Ignoring malformed UDP packet: {len(data)} bytes")
                continue
            self.last_imu = PACKET.unpack(data)
            self.last_receive_time = time.time()


def main():
    args = parse_args()
    rclpy.init()
    node = ImuUdpReceiver(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
