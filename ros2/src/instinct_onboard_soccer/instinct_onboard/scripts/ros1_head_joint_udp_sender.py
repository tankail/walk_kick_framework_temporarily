#!/usr/bin/env python3
"""Forward ROS1 head JointState messages to a local UDP socket."""

import argparse
import socket
import struct
import time

try:
    import rospy
    from sensor_msgs.msg import JointState
except ImportError as exc:
    raise SystemExit("Run this script in a ROS1 environment.") from exc


PACKET = struct.Struct("<dff")


def parse_args():
    parser = argparse.ArgumentParser(description="Forward ROS1 head JointState to UDP for a ROS2 relay.")
    parser.add_argument("--topic", default="/pi_plus_absolute", help="ROS1 JointState topic to subscribe.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15001, help="UDP destination port.")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    rospy.init_node("ros1_head_joint_udp_sender", anonymous=True)
    rospy.loginfo("Forwarding ROS1 %s to udp://%s:%d", args.topic, args.host, args.port)

    def callback(msg):
        positions = dict(zip(msg.name, msg.position))
        if "head_yaw_joint" not in positions or "head_pitch_joint" not in positions:
            return
        packet = PACKET.pack(
            time.time(),
            float(positions["head_yaw_joint"]),
            float(positions["head_pitch_joint"]),
        )
        sock.sendto(packet, addr)

    rospy.Subscriber(args.topic, JointState, callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
