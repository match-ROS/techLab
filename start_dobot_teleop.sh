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

rosrun magician_control arduino_joint_knob_control.py _max_delta_per_event:=1 &
sleep 1

rosrun magician_control match_control.py \
  _speed_x:=80.0 _speed_y:=80.0 _speed_z:=30.0 _speed_r:=30.0 \
  _joint_slow_velocity:=20.0 _joint_fast_velocity:=100.0 \
  _joint_jog_min_knob_rate:=1.0 _joint_jog_max_knob_rate:=25.0
