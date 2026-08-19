#!/usr/bin/env python3
"""Forward ROS1 /cmd_vel Twist messages to a local UDP socket."""

import argparse
import socket
import struct
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError as exc:
    raise SystemExit(
        "Run this script in a ROS1 environment, for example: "
        "source /opt/ros/noetic/setup.bash"
    ) from exc


PACKET = struct.Struct("<dfff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Forward ROS1 geometry_msgs/Twist to UDP for a ROS2 relay."
    )
    parser.add_argument("--topic", default="/cmd_vel", help="ROS1 Twist topic to subscribe.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15000, help="UDP destination port.")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    rospy.init_node("ros1_cmd_vel_udp_sender", anonymous=True)
    rospy.loginfo("Forwarding ROS1 %s to udp://%s:%d", args.topic, args.host, args.port)

    def callback(msg):
        packet = PACKET.pack(
            time.time(),
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        )
        sock.sendto(packet, addr)

    rospy.Subscriber(args.topic, Twist, callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
