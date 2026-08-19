#!/usr/bin/env python3
"""Forward ROS1 /imu/data Imu messages to a local UDP socket (ROS2 relay).

用 UDP 转发 IMU, 绕开 ros1_bridge 的 dynamic_bridge(它会全话题桥接,
污染 /cmd_vel 和 /pi_plus_absolute 等, 导致命令被覆盖)。
"""

import argparse
import socket
import struct
import time

try:
    import rospy
    from sensor_msgs.msg import Imu
except ImportError as exc:
    raise SystemExit(
        "Run this script in a ROS1 environment, for example: "
        "source /opt/ros/noetic/setup.bash"
    ) from exc


# stamp(d) + orientation(wxyz, 4f) + ang_vel(xyz, 3f) + lin_acc(xyz, 3f)
PACKET = struct.Struct("<d10f")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Forward ROS1 sensor_msgs/Imu to UDP for a ROS2 relay."
    )
    parser.add_argument("--topic", default="/imu/data", help="ROS1 Imu topic to subscribe.")
    parser.add_argument("--host", default="127.0.0.1", help="UDP destination host.")
    parser.add_argument("--port", type=int, default=15008, help="UDP destination port.")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    rospy.init_node("ros1_imu_udp_sender", anonymous=True)
    rospy.loginfo("Forwarding ROS1 %s to udp://%s:%d", args.topic, args.host, args.port)

    def callback(msg):
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        packet = PACKET.pack(
            time.time(),
            q.w, q.x, q.y, q.z,
            av.x, av.y, av.z,
            la.x, la.y, la.z,
        )
        sock.sendto(packet, addr)

    rospy.Subscriber(args.topic, Imu, callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
