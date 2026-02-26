#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class DobotF710Teleop:
    def __init__(self):
        # ---- Params ----
        self.topic_joy = rospy.get_param("~joy_topic", "/joy")
        self.topic_cmd = rospy.get_param("~cmd_topic", "/end_effector_coord")

        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.deadzone = float(rospy.get_param("~deadzone", 0.08))

        # Increment per cycle at full stick deflection (units match Dobot API, typically mm / deg)
        self.step_x = float(rospy.get_param("~step_x", 0.8))
        self.step_y = float(rospy.get_param("~step_y", 0.8))
        self.step_z = float(rospy.get_param("~step_z", 0.6))
        self.step_r = float(rospy.get_param("~step_r", 0.8))

        # D-Pad speed trim
        self.trim_step = float(rospy.get_param("~trim_step", 0.1))
        self.step_min = float(rospy.get_param("~step_min", 0.1))
        self.step_max = float(rospy.get_param("~step_max", 5.0))
        self.trim_debounce_s = float(rospy.get_param("~trim_debounce_s", 0.2))

        # Workspace clamps (optional, helps avoid alarms)
        self.clamp_x = rospy.get_param("~clamp_x", [0.0, 300.0])   # [min,max]
        self.clamp_y = rospy.get_param("~clamp_y", [-200.0, 200.0])
        self.clamp_z = rospy.get_param("~clamp_z", [-50.0, 200.0])
        self.clamp_r = rospy.get_param("~clamp_r", [-360.0, 360.0])

        # Start pose
        self.start_pose = rospy.get_param("~start_pose", [200.0, 0.0, 0.0, 0.0])

        # ---- State ----
        self.x, self.y, self.z, self.r = map(float, self.start_pose)
        self.vacuum_on = False

        self._last_buttons = []
        self._last_trim_time = 0.0
        self._joy = None

        # ---- ROS ----
        self.pub = rospy.Publisher(self.topic_cmd, Twist, queue_size=1)
        self.sub = rospy.Subscriber(self.topic_joy, Joy, self._joy_cb, queue_size=1)

        rospy.loginfo("Dobot F710 teleop ready. Publishing to %s", self.topic_cmd)

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def _axis(self, axes, idx, default=0.0):
        if axes is None or idx < 0 or idx >= len(axes):
            return default
        v = float(axes[idx])
        return 0.0 if abs(v) < self.deadzone else v

    def _button_edge(self, buttons, idx):
        """True on rising edge."""
        if buttons is None or idx < 0 or idx >= len(buttons):
            return False
        if not self._last_buttons:
            self._last_buttons = [0] * len(buttons)
        prev = self._last_buttons[idx] if idx < len(self._last_buttons) else 0
        cur = buttons[idx]
        return (prev == 0 and cur == 1)

    def _update_last_buttons(self, buttons):
        if buttons is None:
            return
        self._last_buttons = list(buttons)

    def _joy_cb(self, msg):
        self._joy = msg  # store latest; processed in loop

    def _apply_trim(self, a6, a7, now):
        """
        According to your mapping:
          axis[7] =  1 -> increase speed in x
          axis[7] = -1 -> decrease speed in x
          axis[6] =  1 -> decrease speed in y
          axis[6] = -1 -> increase speed in y
        """
        if (now - self._last_trim_time) < self.trim_debounce_s:
            return

        changed = False
        if a7 == 1.0:
            self.step_x = self._clamp(self.step_x + self.trim_step, self.step_min, self.step_max)
            changed = True
        elif a7 == -1.0:
            self.step_x = self._clamp(self.step_x - self.trim_step, self.step_min, self.step_max)
            changed = True

        if a6 == -1.0:
            self.step_y = self._clamp(self.step_y + self.trim_step, self.step_min, self.step_max)
            changed = True
        elif a6 == 1.0:
            self.step_y = self._clamp(self.step_y - self.trim_step, self.step_min, self.step_max)
            changed = True

        if changed:
            self._last_trim_time = now
            rospy.loginfo("Step sizes: step_x=%.3f step_y=%.3f step_z=%.3f step_r=%.3f",
                          self.step_x, self.step_y, self.step_z, self.step_r)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self._joy is None:
                rate.sleep()
                continue

            axes = self._joy.axes
            buttons = self._joy.buttons

            # --- Buttons ---
            # Buttons[2] = X toggles vacuum
            if self._button_edge(buttons, 2):
                self.vacuum_on = not self.vacuum_on
                rospy.loginfo("Vacuum: %s", "ON" if self.vacuum_on else "OFF")

            # Buttons[6] -> go to start pose [200,0,0,0]
            if self._button_edge(buttons, 6):
                self.x, self.y, self.z, self.r = map(float, self.start_pose)
                rospy.loginfo("Go to start pose: %s", self.start_pose)

            # --- Speed trim via D-Pad axes ---
            # Note: Many drivers give exactly {-1,0,1} here; we treat with rounding.
            a6_raw = self._axis(axes, 6, 0.0)
            a7_raw = self._axis(axes, 7, 0.0)
            a6 = float(int(round(a6_raw))) if a6_raw != 0.0 else 0.0
            a7 = float(int(round(a7_raw))) if a7_raw != 0.0 else 0.0
            self._apply_trim(a6, a7, rospy.get_time())

            # --- Motion axes (your mapping) ---
            # left stick: Axes[0] -> y, Axes[1] -> x
            # Axis[4] -> z, Axis[3] -> rot z
            ay = self._axis(axes, 0, 0.0)
            ax = self._axis(axes, 1, 0.0)
            az = self._axis(axes, 4, 0.0)
            ar = self._axis(axes, 3, 0.0)

            # Incremental update (per loop)
            self.y += self.step_y * ay
            self.x += self.step_x * ax
            self.z += self.step_z * az
            self.r += self.step_r * ar

            # Clamp
            self.x = self._clamp(self.x, self.clamp_x[0], self.clamp_x[1])
            self.y = self._clamp(self.y, self.clamp_y[0], self.clamp_y[1])
            self.z = self._clamp(self.z, self.clamp_z[0], self.clamp_z[1])
            self.r = self._clamp(self.r, self.clamp_r[0], self.clamp_r[1])

            # Publish command
            cmd = Twist()
            cmd.linear.x = self.x
            cmd.linear.y = self.y
            cmd.linear.z = self.z
            cmd.angular.z = self.r

            # manufacturer general_control reads vacuum from angular.x
            cmd.angular.x = 1.0 if self.vacuum_on else 0.0

            self.pub.publish(cmd)

            self._update_last_buttons(buttons)
            rate.sleep()

def main():
    rospy.init_node("dobot_f710_teleop")
    node = DobotF710Teleop()
    node.spin()

if __name__ == "__main__":
    main()