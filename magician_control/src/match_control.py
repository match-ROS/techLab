#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

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
        self.input_lockout_s = float(rospy.get_param("~input_lockout_s", 2.0))
        self.knob_control_mode = str(rospy.get_param("~knob_control_mode", "jog"))
        self.joint_command_interval_s = float(rospy.get_param("~joint_command_interval_s", 0.05))
        self.joint_slow_velocity = float(rospy.get_param("~joint_slow_velocity", 20.0))
        self.joint_fast_velocity = float(rospy.get_param("~joint_fast_velocity", 100.0))
        self.joint_fast_delta = float(rospy.get_param("~joint_fast_delta", 8.0))
        self.joint_acceleration = float(rospy.get_param("~joint_acceleration", 300.0))
        self.joint_min_deg = rospy.get_param("~joint_min_deg", [-135.0, -5.0, -10.0, -145.0])
        self.joint_max_deg = rospy.get_param("~joint_max_deg", [135.0, 85.0, 95.0, 145.0])
        self.joint_jog_min_knob_rate = float(rospy.get_param("~joint_jog_min_knob_rate", 1.0))
        self.joint_jog_max_knob_rate = float(rospy.get_param("~joint_jog_max_knob_rate", 35.0))
        self.joint_jog_timeout_s = float(rospy.get_param("~joint_jog_timeout_s", 0.16))
        self.joint_jog_switch_interval_s = float(rospy.get_param("~joint_jog_switch_interval_s", 0.04))
        self.joint_jog_velocity_update_min_s = float(rospy.get_param("~joint_jog_velocity_update_min_s", 0.04))
        self.gripper_knob_id = str(rospy.get_param("~gripper_knob_id", "P4"))
        self.knob_button_debounce_s = float(rospy.get_param("~knob_button_debounce_s", 0.4))

        self.conveyor_index = int(rospy.get_param("~conveyor_index", 0))
        self.conveyor_max_speed = int(rospy.get_param("~conveyor_max_speed", 8000))  # Puls/s (typisch), ggf. anpassen
        self.conveyor_dead = float(rospy.get_param("~conveyor_dead", 0.05))
        self._last_conveyor_speed = 0.0

        self.start_pose = rospy.get_param("~start_pose", [200.0, 0.0, 0.0, 0.0])

        self._last_trim_t = 0.0
        self._last_buttons = None
        self._joy = None
        self._pending_joint_delta = [0.0, 0.0, 0.0, 0.0]
        self._target_joint_angles = None
        self._last_joint_cmd_t = 0.0
        self._pending_since_t = None
        self._last_ptp_velocity = None
        self._joint_jog_velocity = [0.0, 0.0, 0.0, 0.0]
        self._last_joint_delta_t = [0.0, 0.0, 0.0, 0.0]
        self._last_joint_jog_switch_t = 0.0
        self._last_joint_jog_param_t = 0.0
        self._joint_jog_rr_index = 0
        self._last_knob_button_t = 0.0
        self._active_source = None
        self._source_lock_until = 0.0

        self._last_jog = None  # (mode, cmd) to avoid re-sending
        self.vacuum_on = False

        # in __init__
        self.suck_on = False
        self.grip_on = False

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
        self._apply_ptp_params()

        # optional: home once
        # dType.SetHOMECmd(self.api, temp=0, isQueued=0)

        rospy.Subscriber("/joy", Joy, self._joy_cb, queue_size=1)
        rospy.Subscriber("/joint_angle_delta", Float64MultiArray, self._joint_delta_cb, queue_size=10)
        rospy.Subscriber("/joint_knob_button", String, self._knob_button_cb, queue_size=10)
        rospy.loginfo("Dobot F710 JOG teleop ready.")

    def _joy_cb(self, msg: Joy):
        self._joy = msg

    def _joint_delta_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            rospy.logwarn_throttle(2.0, "Ignoring joint delta with %d values; expected 4.", len(msg.data))
            return
        if not self._accept_source("knob"):
            rospy.loginfo_throttle(1.0, "Ignoring knob command while joystick input is active.")
            return
        if self._pending_since_t is None:
            self._pending_since_t = time.time()
        for i in range(4):
            delta = float(msg.data[i])
            self._pending_joint_delta[i] += delta
            if delta != 0.0:
                self._update_joint_jog_velocity(i, delta)

    def _update_joint_jog_velocity(self, joint_index, delta):
        now = time.time()
        previous_t = self._last_joint_delta_t[joint_index]
        dt = max(now - previous_t, self.joint_jog_velocity_update_min_s) if previous_t > 0.0 else self.joint_jog_velocity_update_min_s
        knob_rate = abs(delta) / dt
        scale = self._clamp(
            (knob_rate - self.joint_jog_min_knob_rate) /
            max(self.joint_jog_max_knob_rate - self.joint_jog_min_knob_rate, 0.001),
            0.0,
            1.0
        )
        velocity = self.joint_slow_velocity + scale * (self.joint_fast_velocity - self.joint_slow_velocity)
        self._joint_jog_velocity[joint_index] = velocity if delta > 0.0 else -velocity
        self._last_joint_delta_t[joint_index] = now

    def _knob_button_cb(self, msg: String):
        device_id = msg.data.strip()
        if device_id != self.gripper_knob_id:
            return

        now = time.time()
        if (now - self._last_knob_button_t) < self.knob_button_debounce_s:
            return
        self._last_knob_button_t = now

        if not self._accept_source("knob"):
            rospy.loginfo_throttle(1.0, "Ignoring knob button while joystick input is active.")
            return

        new = not self.suck_on
        self._set_suck(new)
        rospy.loginfo("Knob %s toggled suction pump: %s", device_id, "on" if new else "off")

    def _accept_source(self, source):
        now = time.time()
        if self._active_source is not None and self._active_source != source and now < self._source_lock_until:
            return False
        self._active_source = source
        self._source_lock_until = now + self.input_lockout_s
        return True

    def _joy_has_activity(self):
        if self._joy is None:
            return False
        motion_axes = (0, 1, 2, 3, 4, 5, 6, 7)
        for idx in motion_axes:
            if self._axis(idx, 0.0) != 0.0:
                return True
        if self._last_buttons is None:
            return any(bool(button) for button in self._joy.buttons)
        for idx, button in enumerate(self._joy.buttons):
            prev = self._last_buttons[idx] if idx < len(self._last_buttons) else 0
            if prev == 0 and button == 1:
                return True
        return False

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

    def _apply_ptp_params(self):
        self._set_ptp_joint_velocity(self.joint_slow_velocity)
        dType.SetPTPCommonParams(self.api, 100.0, 100.0, isQueued=0)

    def _set_ptp_joint_velocity(self, velocity):
        velocity = self._clamp(float(velocity), self.joint_slow_velocity, self.joint_fast_velocity)
        if self._last_ptp_velocity is not None and abs(velocity - self._last_ptp_velocity) < 5.0:
            return
        self._last_ptp_velocity = velocity
        dType.SetPTPJointParams(
            self.api,
            velocity, self.joint_acceleration,
            velocity, self.joint_acceleration,
            velocity, self.joint_acceleration,
            velocity, self.joint_acceleration,
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

    def _send_jog(self, cmd, is_joint=0):
        # cmd is one of dType.JC.*
        jog = (int(is_joint), cmd)
        if jog == self._last_jog:
            return
        dType.SetJOGCmd(self.api, int(is_joint), cmd, isQueued=0)
        self._last_jog = jog

    def _get_joint_angles(self):
        pose = dType.GetPose(self.api)
        return [float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7])]

    def _clamp_joint_angles(self, angles):
        return [
            self._clamp(float(angles[i]), float(self.joint_min_deg[i]), float(self.joint_max_deg[i]))
            for i in range(4)
        ]

    def _send_joint_target(self, angles):
        self._send_jog(dType.JC.JogIdle)
        dType.SetPTPCmd(
            self.api,
            dType.PTPMode.PTPMOVJANGLEMode,
            angles[0], angles[1], angles[2], angles[3],
            isQueued=0
        )

    def _consume_joint_deltas(self):
        if not any(abs(x) > 0.0 for x in self._pending_joint_delta):
            return False

        now = time.time()
        if (now - self._last_joint_cmd_t) < self.joint_command_interval_s:
            return True

        if self._target_joint_angles is None:
            self._target_joint_angles = self._get_joint_angles()

        delta = list(self._pending_joint_delta)
        self._pending_joint_delta = [0.0, 0.0, 0.0, 0.0]
        self._pending_since_t = None
        max_delta = max(abs(x) for x in delta)
        scale = self._clamp(max_delta / self.joint_fast_delta, 0.0, 1.0)
        joint_velocity = self.joint_slow_velocity + scale * (self.joint_fast_velocity - self.joint_slow_velocity)
        self._set_ptp_joint_velocity(joint_velocity)

        self._target_joint_angles = self._clamp_joint_angles([
            self._target_joint_angles[i] + delta[i]
            for i in range(4)
        ])
        self._last_joint_cmd_t = now
        rospy.loginfo("Joint target (deg): %s vel=%.1f delta=%s",
                      [round(x, 2) for x in self._target_joint_angles],
                      self._last_ptp_velocity,
                      [round(x, 2) for x in delta])
        self._send_joint_target(self._target_joint_angles)
        return True

    def _consume_joint_jog(self):
        now = time.time()
        active = []
        for i, last_t in enumerate(self._last_joint_delta_t):
            if last_t > 0.0 and (now - last_t) <= self.joint_jog_timeout_s:
                active.append(i)
            else:
                self._joint_jog_velocity[i] = 0.0

        if not active:
            if self._last_jog is not None and self._last_jog[0] == 1:
                self._send_jog(dType.JC.JogIdle, is_joint=1)
            return False

        if (now - self._last_joint_jog_param_t) >= self.joint_jog_velocity_update_min_s:
            speeds = [abs(v) for v in self._joint_jog_velocity]
            dType.SetJOGJointParams(
                self.api,
                speeds[0], self.joint_acceleration,
                speeds[1], self.joint_acceleration,
                speeds[2], self.joint_acceleration,
                speeds[3], self.joint_acceleration,
                isQueued=0
            )
            self._last_joint_jog_param_t = now

        if (now - self._last_joint_jog_switch_t) >= self.joint_jog_switch_interval_s:
            self._joint_jog_rr_index = (self._joint_jog_rr_index + 1) % len(active)
            self._last_joint_jog_switch_t = now

        joint_index = active[self._joint_jog_rr_index % len(active)]
        positive_cmds = [dType.JC.JogAPPressed, dType.JC.JogBPPressed, dType.JC.JogCPPressed, dType.JC.JogDPPressed]
        negative_cmds = [dType.JC.JogANPressed, dType.JC.JogBNPressed, dType.JC.JogCNPressed, dType.JC.JogDNPressed]
        cmd = positive_cmds[joint_index] if self._joint_jog_velocity[joint_index] > 0.0 else negative_cmds[joint_index]
        self._send_jog(cmd, is_joint=1)
        rospy.loginfo_throttle(
            0.5,
            "Joint JOG active velocities (deg/s): %s",
            [round(v, 1) for v in self._joint_jog_velocity]
        )
        return True

    def _consume_joint_control(self):
        if self.knob_control_mode == "jog":
            return self._consume_joint_jog()
        return self._consume_joint_deltas()

    def _set_suck(self, on: bool):
        dType.SetEndEffectorSuctionCup(self.api, True, bool(on), isQueued=0)
        self.suck_on = bool(on)

    def _set_grip(self, on: bool):
        dType.SetEndEffectorGripper(self.api, True, bool(on), isQueued=0)
        self.grip_on = bool(on)

    def spin(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self._joy is None:
                rate.sleep()
                continue

            if self._consume_joint_control():
                self._last_buttons = list(self._joy.buttons)
                rate.sleep()
                continue

            if self._joy_has_activity():
                if not self._accept_source("joystick"):
                    self._send_jog(dType.JC.JogIdle)
                    self._last_buttons = list(self._joy.buttons)
                    rate.sleep()
                    continue
                self._target_joint_angles = None

            # Buttons
            # in spin(): Buttons[2]=X, Buttons[0]=A

            # X toggles suction; disables gripper
            if self._button_edge(2):
                new = not self.suck_on
                if new:
                    self._set_grip(False)
                self._set_suck(new)

            # A toggles gripper; disables suction
            if self._button_edge(0):
                new = not self.grip_on
                if new:
                    self._set_suck(False)
                self._set_grip(new)

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

            #Förderband
            # Triggers: unpressed=+1, pressed=-1
            lt = self._axis(2, 1.0)  # left trigger
            rt = self._axis(5, 1.0)  # right trigger

            lt_strength = max(0.0, (1.0 - lt) / 2.0)  # 0..1
            rt_strength = max(0.0, (1.0 - rt) / 2.0)

            # decide direction
            cmd_strength = rt_strength - lt_strength   # + forward, - reverse
            if abs(cmd_strength) < self.conveyor_dead:
                speed = 0
            else:
                speed = int(cmd_strength * self.conveyor_max_speed)

            # send only if changed
            if speed != self._last_conveyor_speed:
                if speed == 0:
                    dType.SetEMotor(self.api, self.conveyor_index, 0, 0, isQueued=0)  # disable
                else:
                    dType.SetEMotor(self.api, self.conveyor_index, 1, speed, isQueued=0)  # enable + speed (sign = direction)
                self._last_conveyor_speed = speed


            rate.sleep()

def main():
    rospy.init_node("dobot_f710_jog")
    node = DobotF710Jog()
    node.spin()

if __name__ == "__main__":
    main()
