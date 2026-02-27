#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
from sensor_msgs.msg import Joy

import DobotDllType as dType  # from Dobot SDK

class DobotF710Jog:
    def __init__(self):
        # --- params ---
        self.deadzone = rospy.get_param("~deadzone", 0.08)
        self.speed_x = float(rospy.get_param("~speed_x", 50.0))   # mm/s
        self.speed_y = float(rospy.get_param("~speed_y", 50.0))
        self.speed_z = float(rospy.get_param("~speed_z", 30.0))
        self.speed_r = float(rospy.get_param("~speed_r", 30.0))   # deg/s
        self.acc_lin = float(rospy.get_param("~acc_lin", 100.0))  # mm/s^2
        self.acc_rot = float(rospy.get_param("~acc_rot", 100.0))  # deg/s^2

        self.trim_step = float(rospy.get_param("~trim_step", 5.0))
        self.trim_debounce_s = float(rospy.get_param("~trim_debounce_s", 0.2))
        self.speed_min = float(rospy.get_param("~speed_min", 5.0))
        self.speed_max = float(rospy.get_param("~speed_max", 200.0))

        self.start_pose = rospy.get_param("~start_pose", [200.0, 0.0, 0.0, 0.0])

        self._last_trim_t = 0.0
        self._last_buttons = None
        self._joy = None

        self._last_jog = None  # (mode, cmd) to avoid re-sending
        self.vacuum_on = False

        # --- Dobot connect ---
        self.api = dType.load()
        port = "/dev/ttyACM0"  # oder /dev/serial/by-id/...
        state = dType.ConnectDobot(self.api, port, 115200)[0]
        if state != dType.DobotConnect.DobotConnect_NoError:
            raise RuntimeError(f"ConnectDobot failed: {state}")

        # optional: arm orientation (0=Lefty, 1=Righty) – nur setzen wenn du sicher bist
        # dType.SetArmOrientation(self.api, dType.ArmOrientation.LeftyArmOrientation, isQueued=0)

        # HOME params (deine gewünschte Referenzpose)
        dType.SetHOMEParams(self.api,
                            self.start_pose[0], self.start_pose[1], self.start_pose[2], self.start_pose[3],
                            isQueued=0)

        # Referenzfahrt / Homing (blockierend, bis fertig)
        dType.SetHOMECmdEx(self.api, temp=0.0, isQueued=0)

        # smoother jog behavior
        self._apply_jog_params()

        # optional: home once
        # dType.SetHOMECmd(self.api, temp=0, isQueued=0)

        rospy.Subscriber("/joy", Joy, self._joy_cb, queue_size=1)
        rospy.loginfo("Dobot F710 JOG teleop ready.")

    def _joy_cb(self, msg: Joy):
        self._joy = msg

    def _axis(self, idx, default=0.0):
        if self._joy is None or idx >= len(self._joy.axes):
            return default
        v = float(self._joy.axes[idx])
        return 0.0 if abs(v) < self.deadzone else v

    def _button_edge(self, idx):
        if self._joy is None or idx >= len(self._joy.buttons):
            return False
        if self._last_buttons is None:
            self._last_buttons = [0] * len(self._joy.buttons)
        prev = self._last_buttons[idx]
        cur = self._joy.buttons[idx]
        return (prev == 0 and cur == 1)

    def _apply_jog_params(self):
        dType.SetJOGCoordinateParams(
            self.api,
            self.speed_x, self.acc_lin,
            self.speed_y, self.acc_lin,
            self.speed_z, self.acc_lin,
            self.speed_r, self.acc_rot,
            isQueued=0
        )

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _trim_speeds(self):
        now = time.time()
        if (now - self._last_trim_t) < self.trim_debounce_s:
            return

        a6 = self._axis(6, 0.0)
        a7 = self._axis(7, 0.0)
        a6 = int(round(a6)) if a6 != 0.0 else 0
        a7 = int(round(a7)) if a7 != 0.0 else 0

        changed = False
        # axis[7] = 1 increase x speed, -1 decrease
        if a7 == 1:
            self.speed_x = self._clamp(self.speed_x + self.trim_step, self.speed_min, self.speed_max); changed = True
        elif a7 == -1:
            self.speed_x = self._clamp(self.speed_x - self.trim_step, self.speed_min, self.speed_max); changed = True

        # axis[6] = 1 decrease y speed, -1 increase
        if a6 == -1:
            self.speed_y = self._clamp(self.speed_y + self.trim_step, self.speed_min, self.speed_max); changed = True
        elif a6 == 1:
            self.speed_y = self._clamp(self.speed_y - self.trim_step, self.speed_min, self.speed_max); changed = True

        if changed:
            self._last_trim_t = now
            self._apply_jog_params()
            rospy.loginfo("Jog speeds: vx=%.1f vy=%.1f vz=%.1f vr=%.1f",
                          self.speed_x, self.speed_y, self.speed_z, self.speed_r)

    def _send_jog(self, cmd):
        # cmd is one of dType.JC.*
        if cmd == self._last_jog:
            return
        dType.SetJOGCmd(self.api, 0, cmd, isQueued=0)  # 0 => Cartesian JOG
        self._last_jog = cmd

    def spin(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self._joy is None:
                rate.sleep()
                continue

            # Buttons
            if self._button_edge(2):  # X toggles suction
                self.vacuum_on = not self.vacuum_on
                dType.SetEndEffectorSuctionCup(self.api, self.vacuum_on, True, isQueued=0)
                rospy.loginfo("Vacuum: %s", "ON" if self.vacuum_on else "OFF")

            if self._button_edge(6):  # back to start pose
                # STOP jog first
                self._send_jog(dType.JC.JogIdle)
                x, y, z, r = map(float, self.start_pose)
                self._send_jog(dType.JC.JogIdle)
                dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued=0)

            # Speed trim
            self._trim_speeds()

            # Axes mapping (your mapping)
            y = self._axis(0, 0.0)  # y
            x = self._axis(1, 0.0)  # x
            z = self._axis(4, 0.0)  # z
            r = self._axis(3, 0.0)  # rot z

            # Pick dominant axis to avoid conflicting JOG commands
            vals = [(abs(x), "x", x), (abs(y), "y", y), (abs(z), "z", z), (abs(r), "r", r)]
            vals.sort(reverse=True, key=lambda t: t[0])
            mag, axis, sign = vals[0]

            if mag == 0.0:
                self._send_jog(dType.JC.JogIdle)
            else:
                if axis == "x":
                    cmd = dType.JC.JogAPPressed if sign > 0 else dType.JC.JogANPressed
                elif axis == "y":
                    cmd = dType.JC.JogBPPressed if sign > 0 else dType.JC.JogBNPressed
                elif axis == "z":
                    cmd = dType.JC.JogCPPressed if sign > 0 else dType.JC.JogCNPressed
                else:  # r
                    cmd = dType.JC.JogDPPressed if sign > 0 else dType.JC.JogDNPressed

                self._send_jog(cmd)

            self._last_buttons = list(self._joy.buttons)
            rate.sleep()

def main():
    rospy.init_node("dobot_f710_jog")
    node = DobotF710Jog()
    node.spin()

if __name__ == "__main__":
    main()