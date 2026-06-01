#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
from pydobot import Dobot
from serial.tools import list_ports
from threading import Lock

####################################################################################
# This node is for controlling the robot arm's end effector coordinate (real), 
# and providing the feedback of joint states to the rviz simulation.
####################################################################################

initial_coord = [200, 0, 0, 0]
coord_command = initial_coord
vacuumPumpOn = False
command_lock = Lock()
pending_joint_delta = [0.0, 0.0, 0.0, 0.0]
last_input_source = None
source_lock_until = 0.0
input_lockout_s = 2.0
target_joint_angles = None


def accept_input(source):
    global last_input_source, source_lock_until
    now = rospy.get_time()
    if last_input_source is not None and last_input_source != source and now < source_lock_until:
        return False
    last_input_source = source
    source_lock_until = now + input_lockout_s
    return True


def clamp_joint_angles(angles):
    joint_min_deg = rospy.get_param("~joint_min_deg", [-135.0, -5.0, -10.0, -145.0])
    joint_max_deg = rospy.get_param("~joint_max_deg", [135.0, 85.0, 95.0, 145.0])
    return [
        max(float(joint_min_deg[i]), min(float(joint_max_deg[i]), float(angles[i])))
        for i in range(4)
    ]

def publish_joint_states():
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    # coord_pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=1)
    rate = rospy.Rate(100)  # Set the publishing rate to 100 Hz
    prev_joint_positions = None
    global coord_command, pending_joint_delta, target_joint_angles
    prev_command = coord_command

    def has_joint_changed(current_pose):
        nonlocal prev_joint_positions
        if prev_joint_positions is None:
            return True
        # Compare current joint positions with previous ones
        for i in range(0, 4):  # Assuming joints 0 to 4 (adjust as needed)
            if abs(current_pose[i] - prev_joint_positions[i]) > 0.01:  # Tolerance for change
                return True
        return False
    def has_coordinate_changed(current_pose, prev_command):
        if prev_command is None:
            return True

        # Compare current joint positions with previous ones
        for i in range(0, 4):  # Assuming joints 0 to 4 (adjust as needed)
            if abs(current_pose[i] - prev_command[i]) > 0.01:  # Tolerance for change
                return True
        return False

    while not rospy.is_shutdown():
        with command_lock:
            coord_command = [round(x, 4) for x in coord_command]
            local_coord_command = list(coord_command)
            local_joint_delta = list(pending_joint_delta)
            pending_joint_delta = [0.0, 0.0, 0.0, 0.0]

        joint_command_sent = any(abs(x) > 0.0 for x in local_joint_delta)
        if joint_command_sent:
            pose = arm.get_pose().joints
            if target_joint_angles is None:
                target_joint_angles = [pose.j1, pose.j2, pose.j3, pose.j4]

            target_joint_angles = clamp_joint_angles([
                target_joint_angles[i] + local_joint_delta[i]
                for i in range(4)
            ])
            rospy.loginfo("Joint target (deg): %s", [round(x, 2) for x in target_joint_angles])
            arm.rotate_joint(*target_joint_angles)
            prev_command = local_coord_command

        if not joint_command_sent and has_coordinate_changed(local_coord_command, prev_command):
            target_joint_angles = None
            # print the pose in 2 decimal places
            rospy.loginfo(f"End effector target: {[round(x, 2) for x in local_coord_command]}")
            arm.move_to(local_coord_command[0], local_coord_command[1], local_coord_command[2], local_coord_command[3])
            
            alarms = arm.get_alarms()
            # set a timeout for the alarms
            timeout = 5
            start_time = rospy.get_time()

            # If have alarm and not timeout, reset to previous positions 
            while len(alarms) != 0 and rospy.get_time() - start_time < timeout:
                # arm.clear_alarms()
                print(f"The new pose of coord {local_coord_command} has alarms: {alarms}")
                print(f"Resetting the previous positions to {prev_command}")
                arm.move_to(prev_command[0], prev_command[1], prev_command[2], prev_command[3])
                # rospy.sleep(0.2)  # Wait for the arm to move
                # arm.clear_alarms()
                alarms = arm.get_alarms()
            # If still have alarms after timeout, restore initial positions
            if len(alarms) != 0:
                arm.clear_alarms()
                print(f"Restore initial coord: {initial_coord}")
                arm.move_to(initial_coord[0], initial_coord[1], initial_coord[2], initial_coord[3])
                alarms = arm.get_alarms()
                with command_lock:
                    coord_command = initial_coord
                local_coord_command = initial_coord

            prev_command = local_coord_command
        # else:
            # rospy.loginfo(f"new coord vs prev coord: {coord_command} vs {prev_command}")
        
        # Update the vacuum pump state
        # arm.suck(bool(vacuumPumpOn))      # for the suction cup
        arm.grip(bool(vacuumPumpOn))      # for the gripper
        
        # Publish the coord command, because it maybe updated by other nodes
        # twist = Twist()
        # twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = coord_command
        # twist.angular.x = vacuumPumpOn
        # coord_pub.publish(twist)

        pose = arm.get_pose().joints
        # Check if joint positions have changed
        if prev_joint_positions is None or has_joint_changed(pose):
            # sim follows the real robot's joint states
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_5', 'joint_6', 'joint_7']  # Replace with your joint names
            joint_state_msg.position = [math.radians(pose.j1),
                                         math.radians(pose.j2),
                                         math.radians(90 - pose.j3 + pose.j2),
                                        #  math.radians(pose.j4),
                                         0,
                                         0]  # Replace with actual joint positions
            joint_state_msg.position[3] = (joint_state_msg.position[2] - joint_state_msg.position[1]) - 1.570796325/2 + 0.185   # 1.85 is just a constant offset (can tune)
            joint_state_msg.velocity = []  # Replace with actual joint velocities
            joint_state_msg.effort = []  # Replace with actual joint efforts

            joint_pub.publish(joint_state_msg)
            prev_joint_positions = pose
    

        rate.sleep()

def move_robot_callback(msg):
    # global prev_pose
    global coord_command, vacuumPumpOn
    if not accept_input("joystick"):
        rospy.loginfo_throttle(1.0, "Ignoring joystick command while knob input is active.")
        return

    with command_lock:
        coord_command = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
        vacuumPumpOn = msg.angular.x

        # go to initial coord if coord_command is 0
        if coord_command[0] == 0 and coord_command[1] == 0 and coord_command[2] == 0 and coord_command[3] == 0:
            coord_command = [200, 0, 0, 0]


def joint_delta_callback(msg):
    global pending_joint_delta
    if not accept_input("knob"):
        rospy.loginfo_throttle(1.0, "Ignoring knob command while joystick input is active.")
        return

    if len(msg.data) < 4:
        rospy.logwarn_throttle(2.0, "Ignoring joint delta command with %d values; expected 4.", len(msg.data))
        return

    with command_lock:
        for i in range(4):
            pending_joint_delta[i] += float(msg.data[i])


if __name__ == '__main__':
    rospy.init_node('general_control_node')
    input_lockout_s = float(rospy.get_param("~input_lockout_s", 2.0))

    # Find available ports
    available_ports = list_ports.comports()
    print(f'Available ports: {[x.device for x in available_ports]}')

    # `sudo chown -R $USER:$USER /dev/*`
    port = rospy.get_param("~dobot_port", "")
    if port:
        print(f"Selected configured Dobot port: {port}")
    else:
        port = None
        port_hint = rospy.get_param("~dobot_port_hint", "ttyACM")
        for port_info in available_ports:
            if port_hint in port_info.device:
                print(f"Selected port: {port_info.device}")
                port = port_info.device
                break
    if port is None:
        raise IOError("No Dobot port found.")
    
    global arm
    arm = Dobot(port=port)

    arm.suck(False)
    arm.grip(False)
    # arm.move_to(180.0, 0, 50, 0)

    arm.home()

    global prev_pose
    prev_pose = None
    rospy.Subscriber('/end_effector_coord', Twist, move_robot_callback, queue_size=1)
    rospy.Subscriber('/joint_angle_delta', Float64MultiArray, joint_delta_callback, queue_size=10)
    publish_joint_states()
