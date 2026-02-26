#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from pydobot import Dobot
from serial.tools import list_ports
import math

class DobotF710Direct:
    def __init__(self):
        # ---- Params ----
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))  # robot command rate
        self.deadzone = float(rospy.get_param("~deadzone", 0.08))

        self.start_pose = list(map(float, rospy.get_param("~start_pose", [200.0, 0.0, 0.0, 0.0])))

        # incremental steps per cycle at full stick
        self.step_x = float(rospy.get_param("~step_x", 0.8))
        self.step_y = float(rospy.get_param("~step_y", 0.8))
        self.step_z = float(rospy.get_param("~step_z", 0.6))
        self.step_r = float(rospy.get_param("~step_r", 0.8))

        # trim
        self.trim_step = float(rospy.get_param("~trim_step", 0.1))
        self.step_min = float(rospy.get_param("~step_min", 0.1))
        self.step_max = float(rospy.get_param("~step_max", 5.0))
        self.trim_debounce_s = float(rospy.get_param("~trim_debounce_s", 0.2))

        # thresholds to avoid micro-moves (main anti-ruckel lever)
        self.min_dt_cmd = float(rospy.get_param("~min_dt_cmd", 1.0 / self.rate_hz))
        self.pos_eps = float(rospy.get_param("~pos_eps", 0.5))   # mm
        self.rot_eps = float(rospy.get_param("~rot_eps", 0.5))   # deg

        # workspace clamps
        self.clamp_x = rospy.get_param("~clamp_x", [0.0, 300.0])
        self.clamp_y = rospy.get_param("~clamp_y", [-200.0, 200.0])
        self.clamp_z = rospy.get_param("~clamp_z", [-50.0, 200.0])
        self.clamp_r = rospy.get_param("~clamp_r", [-360.0, 360.0])

        # alarm handling
        self.alarm_timeout_s = float(rospy.get_param("~alarm_timeout_s", 2.0))

        # choose output: grip vs suck
        self.use_grip = bool(rospy.get_param("~use_grip", True))

        # ---- State ----
        self.x, self.y, self.z, self.r = self.start_pose
        self.prev_sent = None
        self.last_cmd_time = 0.0
        self.vacuum_on = False
        self.last_buttons = None
        self.last_trim_time = 0.0

        self.joy = None

        # ---- Robot init ----
        port = self._find_dobot_port()
        rospy.loginfo("Using Dobot port: %s", port)
        self.arm = Dobot(port=port)
        self._set_tool(False)
        self.arm.home()

        # ---- ROS ----
        rospy.Subscriber("/joy", Joy, self._joy_cb, queue_size=1)

    def _find_dobot_port(self):
        available_ports = list_ports.comports()
        for p in available_ports:
            if "ttyACM" in p.device:
                return p.device
        raise IOError("No Dobot port found (ttyACM*).")

    def _set_tool(self, on: bool):
        if self.use_grip:
            self.arm.grip(bool(on))
        else:
            self.arm.suck(bool(on))

    def _joy_cb(self, msg: Joy):
        self.joy = msg

    def _axis(self, idx, default=0.0):
        if self.joy is None or idx >= len(self.joy.axes):
            return default
        v = float(self.joy.axes[idx])
        return 0.0 if abs(v) < self.deadzone else v

    def _button_edge(self, idx):
        if self.joy is None or idx >= len(self.joy.buttons):
            return False
        if self.last_buttons is None:
            self.last_buttons = [0] * len(self.joy.buttons)
        prev = self.last_buttons[idx]
        cur = self.joy.buttons[idx]
        return (prev == 0 and cur == 1)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _apply_trim(self, now):
        # axis[7] = 1 increase x speed, -1 decrease
        # axis[6] = 1 decrease y speed, -1 increase
        if (now - self.last_trim_time) < self.trim_debounce_s:
            return

        a6 = self._axis(6, 0.0)
        a7 = self._axis(7, 0.0)
        a6 = int(round(a6)) if a6 != 0.0 else 0
        a7 = int(round(a7)) if a7 != 0.0 else 0

        changed = False
        if a7 == 1:
            self.step_x = self._clamp(self.step_x + self.trim_step, self.step_min, self.step_max)
            changed = True
        elif a7 == -1:
            self.step_x = self._clamp(self.step_x - self.trim_step, self.step_min, self.step_max)
            changed = True

        if a6 == -1:
            self.step_y = self._clamp(self.step_y + self.trim_step, self.step_min, self.step_max)
            changed = True
        elif a6 == 1:
            self.step_y = self._clamp(self.step_y - self.trim_step, self.step_min, self.step_max)
            changed = True

        if changed:
            self.last_trim_time = now
            rospy.loginfo("Steps: x=%.2f y=%.2f z=%.2f r=%.2f", self.step_x, self.step_y, self.step_z, self.step_r)

    def _target_changed_enough(self, target):
        if self.prev_sent is None:
            return True
        dx = abs(target[0] - self.prev_sent[0])
        dy = abs(target[1] - self.prev_sent[1])
        dz = abs(target[2] - self.prev_sent[2])
        dr = abs(target[3] - self.prev_sent[3])
        return (dx > self.pos_eps) or (dy > self.pos_eps) or (dz > self.pos_eps) or (dr > self.rot_eps)

    def _try_move_with_alarm_handling(self, target):
        prev = self.prev_sent if self.prev_sent is not None else self.start_pose

        self.arm.move_to(target[0], target[1], target[2], target[3])

        alarms = self.arm.get_alarms()
        if len(alarms) == 0:
            return True

        start = rospy.get_time()
        while len(alarms) != 0 and (rospy.get_time() - start) < self.alarm_timeout_s:
            rospy.logwarn("Alarm %s -> reverting to prev %s", alarms, prev)
            self.arm.move_to(prev[0], prev[1], prev[2], prev[3])
            alarms = self.arm.get_alarms()

        if len(alarms) != 0:
            rospy.logerr("Alarm persists -> clear + go start %s", self.start_pose)
            self.arm.clear_alarms()
            self.arm.move_to(self.start_pose[0], self.start_pose[1], self.start_pose[2], self.start_pose[3])
            self.x, self.y, self.z, self.r = self.start_pose
            return False

        return True

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.joy is None:
                rate.sleep()
                continue

            now = rospy.get_time()

            # buttons
            if self._button_edge(2):  # X toggles tool
                self.vacuum_on = not self.vacuum_on
                rospy.loginfo("Tool: %s", "ON" if self.vacuum_on else "OFF")
                self._set_tool(self.vacuum_on)

            if self._button_edge(6):  # back to start pose
                self.x, self.y, self.z, self.r = self.start_pose
                rospy.loginfo("Go start pose %s", self.start_pose)

            # trim
            self._apply_trim(now)

            # axes mapping (your mapping)
            ay = self._axis(0, 0.0)  # y
            ax = self._axis(1, 0.0)  # x
            az = self._axis(4, 0.0)  # z
            ar = self._axis(3, 0.0)  # rot z

            # integrate absolute target
            self.y += self.step_y * ay
            self.x += self.step_x * ax
            self.z += self.step_z * az
            self.r += self.step_r * ar

            # clamp
            self.x = self._clamp(self.x, self.clamp_x[0], self.clamp_x[1])
            self.y = self._clamp(self.y, self.clamp_y[0], self.clamp_y[1])
            self.z = self._clamp(self.z, self.clamp_z[0], self.clamp_z[1])
            self.r = self._clamp(self.r, self.clamp_r[0], self.clamp_r[1])

            target = [self.x, self.y, self.z, self.r]

            # send robot command only at min dt and if changed enough
            if (now - self.last_cmd_time) >= self.min_dt_cmd and self._target_changed_enough(target):
                ok = self._try_move_with_alarm_handling(target)
                self.last_cmd_time = now
                if ok:
                    self.prev_sent = target

            # update last buttons
            self.last_buttons = list(self.joy.buttons) if self.joy is not None else self.last_buttons

            rate.sleep()

def main():
    rospy.init_node("dobot_f710_direct")
    node = DobotF710Direct()
    node.spin()

if __name__ == "__main__":
    main()