#!/usr/bin/env bash
# Isolate Soccer ROS2 traffic to this robot. Source this before every ROS2
# terminal: bringup, move_to_zero, and soccer control.

export ROS_DOMAIN_ID=56
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_SOCCER_ROS2_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_URI="file://${_SOCCER_ROS2_ENV_DIR}/cyclonedds_route_c_local.xml"
unset _SOCCER_ROS2_ENV_DIR
