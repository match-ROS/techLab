#!/usr/bin/env bash
set -e

source /opt/ros/one/setup.bash
source ~/catkin_ws/devel/setup.bash

JOY_DEV="/dev/input/js0"

if ! pgrep -x roscore >/dev/null; then
  roscore &
  sleep 2
fi

rosrun joy joy_node _dev:=${JOY_DEV} _autorepeat_rate:=50 &
sleep 1

rosrun magician_control arduino_joint_knob_control.py &
sleep 1

rosrun magician_control match_control.py
