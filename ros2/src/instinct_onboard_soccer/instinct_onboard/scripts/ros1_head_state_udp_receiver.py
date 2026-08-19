#!/usr/bin/env python3
"""Receive ROS2 head feedback over UDP and publish ROS1 dancer_io motor state topics."""

import argparse
import select
import socket
import struct
import time

try:
    import rospy
    try:
        # dancer-io subscribes to livelybot_msg/MotorState on the head feedback topics.
        from livelybot_msg.msg import MotorState
    except ImportError:
        # Keep compatibility with older workspaces that only expose livelybot_serial.
        from livelybot_serial.msg import MotorState
except ImportError as exc:
    raise SystemExit("Run this script in a ROS1 environment with livelybot_serial/livelybot_msg sourced.") from exc


PACKET = struct.Struct("<dffff")


def parse_args():
    parser = argparse.ArgumentParser(description="Publish ROS1 head controller feedback from UDP.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--port", type=int, default=15002, help="UDP port to bind.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish/check rate in Hz.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Seconds before stale feedback is ignored.")
    parser.add_argument(
        "--pitch-topic",
        default="/livelybot_real_real/head_pitch_joint_controller/state",
        help="ROS1 pitch MotorState feedback topic.",
    )
    parser.add_argument(
        "--yaw-topic",
        default="/livelybot_real_real/head_yaw_joint_controller/state",
        help="ROS1 yaw MotorState feedback topic.",
    )
    return parser.parse_args()


def make_state(pos, vel):
    msg = MotorState()
    msg.pos = float(pos)
    msg.vel = float(vel)
    msg.tau = 0.0
    msg.temperature = 0
    return msg


def main():
    args = parse_args()
    rospy.init_node("ros1_head_state_udp_receiver", anonymous=True)

    pitch_pub = rospy.Publisher(args.pitch_topic, MotorState, queue_size=10)
    yaw_pub = rospy.Publisher(args.yaw_topic, MotorState, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.setblocking(False)

    last_feedback = None
    last_receive_time = None
    rate = rospy.Rate(args.rate)
    rospy.loginfo(
        "Publishing udp://%s:%d head feedback to %s and %s",
        args.host,
        args.port,
        args.pitch_topic,
        args.yaw_topic,
    )

    while not rospy.is_shutdown():
        while True:
            readable, _, _ = select.select([sock], [], [], 0.0)
            if not readable:
                break
            data, _ = sock.recvfrom(1024)
            if len(data) != PACKET.size:
                rospy.logwarn("Ignoring malformed UDP packet: %d bytes", len(data))
                continue
            _, yaw, pitch, yaw_vel, pitch_vel = PACKET.unpack(data)
            last_feedback = (yaw, pitch, yaw_vel, pitch_vel)
            last_receive_time = time.time()

        if last_feedback is not None and time.time() - last_receive_time <= args.timeout:
            yaw, pitch, yaw_vel, pitch_vel = last_feedback
            yaw_pub.publish(make_state(yaw, yaw_vel))
            pitch_pub.publish(make_state(pitch, pitch_vel))
        rate.sleep()


if __name__ == "__main__":
    main()
