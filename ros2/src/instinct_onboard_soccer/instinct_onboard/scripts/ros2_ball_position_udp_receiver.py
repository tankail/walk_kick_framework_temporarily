#!/usr/bin/env python3
"""Receive UDP ball_field + ball_velocity packets and publish ROS2 PointStamped + Odometry.

- PointStamped on --topic      (position only, 向后兼容)
- Odometry   on --vel_topic    (position + linear velocity, 供 ballvel 踢球策略 measured 源)
"""

import argparse
import select
import socket
import struct
import time

from geometry_msgs.msg import PointStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
import rclpy
from rclpy.node import Node


PACKET = struct.Struct("<d?ffff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive UDP ball_field + ball_velocity packets and publish ROS2 PointStamped + Odometry."
    )
    parser.add_argument(
        "--topic",
        default="/ros1_ball_position",
        help="ROS2 PointStamped topic to publish (position only).",
    )
    parser.add_argument(
        "--vel_topic",
        default="/soccer/ball_base_xy_velocity",
        help="ROS2 Odometry topic to publish (position + linear velocity).",
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15003, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish/check rate in Hz.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Seconds before stale ball input is treated as unseen.",
    )
    parser.add_argument(
        "--frame-id",
        default="base_link",
        help="Frame id for robot-relative x/y ball position.",
    )
    return parser.parse_args()


class BallPositionUdpReceiver(Node):
    def __init__(self, args):
        super().__init__("ros2_ball_position_udp_receiver")
        self.args = args
        self.publisher = self.create_publisher(PointStamped, args.topic, 10)
        self.vel_publisher = self.create_publisher(Odometry, args.vel_topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host, args.port))
        self.sock.setblocking(False)
        self.last_ball = None
        self.last_receive_time = None
        self.was_visible = False
        self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing UDP udp://{args.host}:{args.port} ball pos+vel to ROS2 "
            f"{args.topic} (PointStamped) + {args.vel_topic} (Odometry); timeout={args.timeout:.2f}s"
        )

    def timer_callback(self):
        self._drain_udp()
        fresh = (
            self.last_ball is not None
            and self.last_receive_time is not None
            and time.time() - self.last_receive_time <= self.args.timeout
        )
        if not fresh:
            if self.was_visible:
                self.get_logger().warn("UDP ball position timed out; publishing nothing.")
                self.was_visible = False
            return

        seen, x, y, vx, vy = self.last_ball
        if not seen:
            if self.was_visible:
                self.get_logger().info("ROS1 VisionInfo reports no ball; publishing nothing.")
                self.was_visible = False
            return

        # PointStamped (position only, 向后兼容)
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.publisher.publish(msg)

        # Odometry (position + velocity) — 供 ballvel 踢球策略 measured 源
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.args.frame_id
        odom.child_frame_id = self.args.frame_id
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = 0.0
        self.vel_publisher.publish(odom)

        if not self.was_visible:
            self.get_logger().info(
                f"Received fresh visible ball pos=({x:.2f},{y:.2f}) vel=({vx:.2f},{vy:.2f})."
            )
            self.was_visible = True

    def _drain_udp(self):
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                return
            data, _ = self.sock.recvfrom(1024)
            if len(data) != PACKET.size:
                self.get_logger().warn(f"Ignoring malformed UDP packet: {len(data)} bytes")
                continue
            _, seen, x, y, vx, vy = PACKET.unpack(data)
            self.last_ball = (bool(seen), float(x), float(y), float(vx), float(vy))
            self.last_receive_time = time.time()


def main():
    args = parse_args()
    rclpy.init()
    node = BallPositionUdpReceiver(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
