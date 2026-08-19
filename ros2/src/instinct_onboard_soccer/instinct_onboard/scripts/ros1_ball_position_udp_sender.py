#!/usr/bin/env python3
"""Forward ROS1 VisionInfo ball_field to UDP for ROS2 consumers."""

import argparse
import socket
import struct
import time

try:
    import rospy
    from dmsgs.msg import VisionInfo
except ImportError as exc:
    raise SystemExit("Run this script in a ROS1 environment with dmsgs sourced.") from exc


PACKET = struct.Struct("<d?ffff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Forward ROS1 dmsgs/VisionInfo ball_field + ball_velocity to UDP."
    )
    parser.add_argument(
        "--topic",
        default="/dvision_1/VisionInfo",
        help="ROS1 VisionInfo topic to subscribe.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15003, help="UDP destination port.")
    parser.add_argument(
        "--scale",
        type=float,
        default=0.01,
        help="Scale applied to ball_field x/y before sending. "
             "ROS1 dbehavior uses centimeters; default converts cm to meters.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    rospy.init_node("ros1_ball_position_udp_sender", anonymous=True)
    rospy.loginfo(
        "Forwarding ROS1 %s ball_field + ball_velocity to udp://%s:%d scale=%g",
        args.topic,
        args.host,
        args.port,
        args.scale,
    )

    def callback(msg):
        # 位置和速度都乘 scale (速度 = Δ位置/Δt, 同尺度)。
        # dvision 的 ball_velocity 由 ball_field 历史窗差分得到, 与 ball_field 同单位。
        packet = PACKET.pack(
            time.time(),
            bool(msg.see_ball),
            float(msg.ball_field.x) * args.scale,
            float(msg.ball_field.y) * args.scale,
            float(msg.ball_velocity.x) * args.scale,
            float(msg.ball_velocity.y) * args.scale,
        )
        sock.sendto(packet, addr)

    rospy.Subscriber(args.topic, VisionInfo, callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
