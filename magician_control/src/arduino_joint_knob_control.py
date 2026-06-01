#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import threading

import rospy
import serial
from serial.tools import list_ports
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


ENCODER_RE = re.compile(r"^(?P<device>[^:]+):ENC:(?P<delta>[-+]?\d+)\s*$")
BUTTON_RE = re.compile(r"^(?P<device>[^:]+):BTN:(?P<state>[01])\s*$")


class ArduinoJointKnobControl:
    def __init__(self):
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.05))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))
        self.step_deg = float(rospy.get_param("~step_deg", 0.25))
        self.joint_count = int(rospy.get_param("~joint_count", 4))
        self.port_keywords = rospy.get_param("~port_keywords", [
            "arduino",
            "esp32",
            "usb jtag",
            "ch340",
            "cp210",
            "wch",
        ])
        self.ports = rospy.get_param("~ports", [])
        self.exclude_ports = set(rospy.get_param("~exclude_ports", []))
        self.device_joint_map = rospy.get_param("~device_joint_map", {
            "P1": 0,
            "P2": 1,
            "P3": 2,
            "P4": 3,
        })

        self._lock = threading.Lock()
        self._pending_delta_deg = [0.0] * self.joint_count
        self._stop_event = threading.Event()
        self._threads = []

        self.pub = rospy.Publisher("/joint_angle_delta", Float64MultiArray, queue_size=5)
        self.button_pub = rospy.Publisher("/joint_knob_button", String, queue_size=5)

    def _discover_ports(self):
        if self.ports:
            return [p for p in self.ports if p not in self.exclude_ports]

        ports = []
        keywords = [str(keyword).lower() for keyword in self.port_keywords]
        for port_info in list_ports.comports():
            metadata = " ".join([
                str(port_info.description or ""),
                str(port_info.manufacturer or ""),
                str(port_info.product or ""),
                str(port_info.hwid or ""),
            ]).lower()
            if any(keyword in metadata for keyword in keywords):
                ports.append(port_info.device)

        return sorted(set(ports) - self.exclude_ports)

    def _reader(self, port):
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                with serial.Serial(port, self.baudrate, timeout=self.serial_timeout) as ser:
                    rospy.loginfo("Arduino knob reader connected on %s", port)
                    while not rospy.is_shutdown() and not self._stop_event.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue
                        self._handle_line(raw.decode("utf-8", errors="replace").strip(), port)
            except serial.SerialException as exc:
                rospy.logwarn_throttle(5.0, "Arduino knob reader cannot open %s: %s", port, exc)
                rospy.sleep(1.0)

    def _handle_line(self, line, port):
        match = ENCODER_RE.match(line)
        if match:
            self._handle_encoder(match, port)
            return

        match = BUTTON_RE.match(line)
        if match:
            device_id = match.group("device")
            if match.group("state") == "1":
                self.button_pub.publish(String(data=device_id))
            return

        if line.endswith(":READY") or line.endswith(":BOOT"):
            rospy.loginfo("Arduino knob %s on %s", line, port)

    def _handle_encoder(self, match, port):
        device_id = match.group("device")
        if device_id not in self.device_joint_map:
            rospy.logwarn_throttle(5.0, "Ignoring unknown knob id %s from %s", device_id, port)
            return

        joint_index = int(self.device_joint_map[device_id])
        if joint_index < 0 or joint_index >= self.joint_count:
            rospy.logwarn_throttle(5.0, "Ignoring knob id %s with invalid joint index %d", device_id, joint_index)
            return

        delta_deg = int(match.group("delta")) * self.step_deg
        with self._lock:
            self._pending_delta_deg[joint_index] += delta_deg

    def start(self):
        ports = self._discover_ports()
        if not ports:
            rospy.logwarn("No Arduino knob serial ports found. Set ~ports if auto-discovery misses them.")
            return

        for port in ports:
            thread = threading.Thread(target=self._reader, args=(port,))
            thread.daemon = True
            thread.start()
            self._threads.append(thread)

    def spin(self):
        self.start()
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            with self._lock:
                delta = self._pending_delta_deg
                self._pending_delta_deg = [0.0] * self.joint_count

            if any(abs(x) > 0.0 for x in delta):
                self.pub.publish(Float64MultiArray(data=delta))

            rate.sleep()

        self._stop_event.set()


def main():
    rospy.init_node("arduino_joint_knob_control")
    ArduinoJointKnobControl().spin()


if __name__ == "__main__":
    main()
