"""
Project Name: Teleoperation interface of Mirte Master
File: controller_script_final.py
Description: Does the reading out and the publishing of commands for the Mirte Master

Author: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk
License: MIT
Repository: https://github.com/QSame9/Teleoperation-for-Mirte-Master

This file is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. 
It was commissioned by the Cognitive Robotics department of the TU Delft.


This file is distributed under the terms
of the MIT License. See the LICENSE file in the root of this repository
or visit https://opensource.org/licenses/MIT for more details.
"""
# ---- Import ----
import rospy
import numpy as np
from sensor_classes_final import Encoder, Joystick, Switch, YawEncoder, GripperEncoder, Joint
from gripper_simulator_file_final import GripperSimulator
import math
import csv
import pandas as pd
import time
# import data_collection_script

# load the specific message format required for joint commands
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
from mirte_msgs.srv import SetServoAngle, GetPinValue, SetPinValue
from mirte_msgs.msg import ServoPosition, Intensity
from sensor_msgs.msg import JointState


# ---- Initialize controller node --------

# start a new ROS node
rospy.init_node('simple_controller_publisher')
# provide on-screen information
rospy.loginfo('The base_controller_publisher has started!')



# ---- Set the publishers and services to control the position of the Mirte Master --------

# create the publisher, tell it which topic to publish on
base_command_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10) 
# create the publisher, tell it which topic to publish on
arm_command_publisher = rospy.Publisher('/arm/joint_position_controller/command', Float64MultiArray, queue_size=10)
#create the service, serviceproxy is aan de client side, dus voor het verzenden
set_gripper_angle_service = rospy.ServiceProxy('/mirte/set_servoGripper_servo_angle', SetServoAngle, persistent = True)
set_pincher_servo_angle = rospy.ServiceProxy('/mirte_remote/mirte/set_gripper_cntlr_servo_angle', SetServoAngle, persistent = True)

# ---- General values --------

update_speed = 0.1
stiffness_gripper_spring = 2.08
yaw_offset = 0.51 #yaw on magnet
shoulder_offset = 2.1 #straight up
elbow_offset = 3.13 - 0.5 #straight up
wrist_offset = 4.50 #fully bent inwards
gripper_offset = 2.20 #fully open
servo_offset = math.radians(130) #graden
spring_engagement_angle = 0.60

def callback_states(msg):
    global joint_states_data
    joint_states_data = msg.position

# ---- define sensors on controller --------

yaw_encoder = YawEncoder('/mirte_remote/mirte/encoder/encoders/a', spring_engagement_angle, 
                        calibrating_angle = 0, scaling_factor = 1, first_order_control_gain = 2, offset = yaw_offset)
schoulder_encoder = Encoder('/mirte_remote/mirte/encoder/encoders/b', 
                        calibrating_angle = 0, offset = shoulder_offset)
elbow_encoder = Encoder('/mirte_remote/mirte/encoder/encoders/c', scaling_factor = -1,
                        calibrating_angle = 0, offset = elbow_offset)
wrist_encoder = Encoder('/mirte_remote/mirte/encoder/encoders/d', scaling_factor = -1,
                        calibrating_angle = 1.28, offset = wrist_offset) 
gripper_encoder = GripperEncoder('/mirte_remote/mirte/encoder/encoders/e', 
                        calibrating_angle = 0, offset = gripper_offset, scaling_factor = 1) #checken
joystick = Joystick('/mirte_remote/mirte/intensity/x_axis', 
                        '/mirte_remote/mirte/intensity/y_axis', 0.5 * 1/2040)
gripper_simulator = GripperSimulator()
#joint_states = rospy.Subscriber('/arm/joint_states', JointState, callback_states)

mirte_yaw_joint_state = Joint('/mirte/servos/servoRot/position')
mirte_schoulder_joint_state = Joint('/mirte/servos/servoShoulder/position')
mirte_elbow_joint_state = Joint('/mirte/servos/servoElbow/position')
mirte_wrist_joint_state = Joint('/mirte/servos/servoWrist/position')
mirte_gripper_joint_state = Joint('/mirte/servos/servoGripper/position')

switch_1 = Switch("20")
switch_2 = Switch("19")
switch_3 = Switch("18")


# ---- define robot zero states --------
base_zero = Twist()
base_zero.linear.x = 0.0
base_zero.linear.y = 0.0
base_zero.angular.z = 0.0

arm_up = Float64MultiArray()
arm_up.data = [0,  0.9547413253784178, -2.0481974124908446, -1.418008804321289]

gripper_open = 0.5
pincher_free = 0
# set_pincher_servo_angle(pincher_free)

# ---- Set everything to start position --------
set_gripper_angle_service(gripper_open) #-0.3 is een vinger ertussen 
arm_command_publisher.publish(arm_up)
rospy.sleep(1)
rospy.loginfo('Robot is ready to go!')

print(f"switch_1.value {switch_1.value.data}, switch_2.value {switch_2.value.data}, switch_3.value {switch_3.value.data},")
rospy.sleep(0.1)

# ---- Calibrate joystick
# print("press enter to calibrate yaw on magnet:")
# ready = input()
# yaw_encoder.calibrate()

# print("press enter to calibrate shoulder straight up:")
# ready = input()
# schoulder_encoder.calibrate()

# print("press enter to calibrate elbow straigt up:")
# ready = input()
# elbow_encoder.calibrate()

# print("press enter to calibrate wrist fully bent inwards:")
# ready = input()
# wrist_encoder.calibrate()

# print("press enter to calibrate gripper fully open:")
# ready = input()
# gripper_encoder.calibrate()

# rospy.sleep(0.2)
print('Controller is calibrated! Wil start controlling now!')

def get_switch_values():
    return switch_1.value.data, switch_2.value.data, switch_3.value.data

rospy.sleep(0.1)

def main_loop():
    time_step_data = {"get switch value":[], "get base input":[], "get arm input": [], "get gripper input":[], "publish base": [], "publish arm": [], "publish gripper": [], "publish servo": [], "sleep":[], "total_step_time": []}
    # ---- Data types for actuation data --------
    arm_state = Float64MultiArray()
    base_speed = Twist()
    loop_frequency = 40
    time_last_switch_check = time.time() - 0.5

    i = 0
    t8 = time.time()
    t0 = time.time()
    while i < 4000:
        i += 1
        time_step_data["total_step_time"].append(time.time() - t0)
        t0 = time.time() 
        time_step_data["sleep"].append(t0 - t8)
        
        if time.time() - time_last_switch_check > 0.5:
            val_switch_1, val_switch_2, val_switch_3 = get_switch_values()
            time_last_switch_check = time.time()

        t1 = time.time()

        #---- get base input --------
        if val_switch_1 == 1:
            base_speed.linear.x = (-joystick.x_value)
            base_speed.linear.y = (joystick.y_value)
            base_speed.angular.z = 0.0 #for holonomic, change into if statement for other switch value
        else:
            base_speed.linear.x = (-joystick.x_value)
            base_speed.linear.y = 0.0
            base_speed.angular.z = 3*(joystick.y_value)
        t2 = time.time()
        #---- get arm input --------
        if val_switch_2 == 1:
            arm_state.data = [yaw_encoder.value, -schoulder_encoder.value, -elbow_encoder.value, -wrist_encoder.value]
        else:
            arm_state.data = [yaw_encoder.value, schoulder_encoder.value, 
                     elbow_encoder.value, wrist_encoder.value]
        t3 = time.time()
        
        

        #print(f"arm_data_sent = {arm_state.data}, arm_data_states = {joint_states_data}")

        #---- get gripper input --------
        pincher_angle = gripper_encoder.value
        actuation_mirte_gripper = gripper_encoder.actuation_mirte_gripper
        

        #---- get mirte gripping force--------
        gripper_simulator.set_gripper_angle(actuation_mirte_gripper)
        gripping_force = gripper_simulator.get_gripping_force()
        desired_spring_displacement = gripping_force / 16* stiffness_gripper_spring
        pincher_servo_angle = math.degrees(-pincher_angle + desired_spring_displacement+servo_offset)
        t4 = time.time()
        
        # print(f"pincher_angle: {pincher_angle}, pincher_servo_angle: {math.radians(pincher_servo_angle)}")

        #add publish command for controller gripper servo
        base_command_publisher.publish(base_speed)
        t5 = time.time()
        arm_command_publisher.publish(arm_state)
        t6 = time.time()
        set_gripper_angle_service(actuation_mirte_gripper)
        t7 = time.time()
        if pincher_servo_angle < 0:
            pincher_servo_angle = 0
        set_pincher_servo_angle(pincher_servo_angle)
        # print(f"pincher_servo_angle{pincher_servo_angle} = \n -pincher_angle {pincher_angle} \n + desired_spring_displacement {desired_spring_displacement} \n + servo_offset {servo_offset}")
        t8 = time.time()
        step_period = time.time() - t0
        if step_period < 1 / loop_frequency:
            rospy.sleep(1 / loop_frequency - step_period)

        time_step_data["get switch value"].append(t1 - t0)
        time_step_data["get base input"].append(t2 - t1)
        time_step_data["get arm input"].append(t3 - t2)
        time_step_data["get gripper input"].append(t4 - t3)
        time_step_data["publish base"].append(t5 - t4)
        time_step_data["publish arm"].append(t6 - t5)
        time_step_data["publish gripper"].append(t7 - t6)
        time_step_data["publish servo"].append(t8 - t7)

    df = pd.DataFrame(time_step_data)
    df.to_csv("time_step_data.csv", index=False)
main_loop()
    


test_data = {"t_yaw_enc": yaw_encoder.timestamps, "data_yaw_enc": yaw_encoder.data,
                   "t_yaw_mirte": mirte_yaw_joint_state.timestamps, "data_yaw_mirte": mirte_yaw_joint_state.data,
                   "t_schoulder_enc": schoulder_encoder.timestamps, "data_schoulder_enc": schoulder_encoder.data,
                   "t_schoulder_mirte": mirte_schoulder_joint_state.timestamps, "data_schoulder_mirte": mirte_schoulder_joint_state.data,
                   "t_elbow_enc": elbow_encoder.timestamps, "data_elbow_enc": elbow_encoder.data,
                   "t_elbow_mirte": mirte_elbow_joint_state.timestamps, "data_elbow_mirte": mirte_elbow_joint_state.data,
                   "t_wrist_enc": wrist_encoder.timestamps, "data_wrist_enc": wrist_encoder.data,
                   "t_wrist_mirte": mirte_wrist_joint_state.timestamps, "data_wrist_mirte": mirte_wrist_joint_state.data,
                   "t_gripper_enc": gripper_encoder.timestamps, "data_gripper_enc": gripper_encoder.data,
                   "t_gripper_mirte": mirte_gripper_joint_state.timestamps, "data_gripper_mirte": mirte_gripper_joint_state.data}
df = pd.DataFrame(dict([(key, pd.Series(value)) for key, value in test_data.items()]))

df.to_csv('joint_data.csv', index=False)


