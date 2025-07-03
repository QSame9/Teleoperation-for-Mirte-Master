"""
Project Name: Teleoperation interface of Mirte Master
File: sensor_classes_final.py
Description: A support file for the 'controller_script_final.py' where the classes 
are defined for both the sensors and actuator commands.

Author: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk
License: MIT
Repository: https://github.com/QSame9/Teleoperation-for-Mirte-Master

This file is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. 
It was commissioned by the Cognitive Robotics department of the TU Delft.


This file is distributed under the terms
of the MIT License. See the LICENSE file in the root of this repository
or visit https://opensource.org/licenses/MIT for more details.
"""

import rospy
import numpy as np
import time
from datetime import datetime

# load the specific message format required for joint commands
from std_msgs.msg import Float32
from mirte_msgs.srv import GetPinValue
from mirte_msgs.msg import Intensity, ServoPosition

t0 = time.time()


class Encoder():
    
    def __init__(self, path, scaling_factor=1, calibrating_angle=None, offset = 0):
        self.subscriber = rospy.Subscriber(path, Float32, self.msg_reciever)
        self._value = None
        self.raw_value = None
        self.calibrating_angle = calibrating_angle
        self.scaling_factor = scaling_factor
        self.offset = offset
        self.raw_data = []
        self.data = []
        self.timestamps = []
        
    def msg_reciever(self, msg): 
        raw_value = msg.data
        timestamp = time.time() - t0
        
        if self.offset is None:
            return
        value = (raw_value - self.offset)
        while value > np.pi:
            value -= 2 * np.pi
        while value < -np.pi:
            value += 2 * np.pi
        value = value * self.scaling_factor
        self.value = value
        self.raw_value = raw_value
        
        self.raw_data.append(raw_value)
        self.data.append(value)
        self.timestamps.append(timestamp)
        
        
    # def calibrate(self):
    #     if self.calibrating_angle is None:
    #         raise ValueError("there is no calibrating angle defined for this encoder")
    #     if self.raw_value is None:
    #         raise ValueError("no encoder data available to calibrate on")
    #     self.offset = self.raw_value - self.calibrating_angle
    #     print('off', self.offset, 'raw', self.raw_value, 'cali', self.calibrating_angle)


    def get_value(self):
        return self._value

    def set_value(self, val):
        self._value = val


    value = property(fget = get_value, fset = set_value)

class GripperEncoder(Encoder):
    def __init__(self, path, scaling_factor=1, calibrating_angle=None, offset = 0):
        super().__init__(path, scaling_factor, calibrating_angle, offset)

    def get_actuation_mirte_gripper(self):
        pincher_range = 2.27
        if not self._value:
            return 0
        return (-self._value * (1/pincher_range) + 0.5)

    actuation_mirte_gripper = property(fget=get_actuation_mirte_gripper) 
    
class YawEncoder(Encoder):
    def __init__(self, path, spring_engagement_angle, scaling_factor=1, calibrating_angle=None, first_order_control_gain = 1, offset = 0):
        super().__init__(path, scaling_factor, calibrating_angle, offset)
        self.scroll_offset = 0
        self.first_order_control_gain = first_order_control_gain
        self.getvalue_timestamp = None
        self.spring_engagement_angle = spring_engagement_angle * scaling_factor
    
    def set_yaw_value(self, val):
        self._value = val

        if self.offset is None:
            return

        if self.getvalue_timestamp is None:
            dt = 0
        else:
            # if datetime.now().second - self.getvalue_timestamp.second > 1:
            #     raise RuntimeError("Too mutch time between yaw encoder value updates")
            dt = 10e-7 * (datetime.now().microsecond - self.getvalue_timestamp.microsecond)
        if dt < 0:
            dt += 1



        self.getvalue_timestamp = datetime.now()

        spring_depression = max(0, abs(self._value) - self.spring_engagement_angle)
        if self._value > 0:
            self.scroll_offset += spring_depression * self.first_order_control_gain * dt
        else:
            self.scroll_offset -= spring_depression * self.first_order_control_gain * dt
        
        self.scroll_offset = min(max(self.scroll_offset, -1), 1)

        

    def get_yaw_value(self):
        if self._value > self.spring_engagement_angle:
            return self.spring_engagement_angle + self.scroll_offset
        elif self._value < -self.spring_engagement_angle:
            return -self.spring_engagement_angle + self.scroll_offset
        # print(f"_value {self._value}, springengagementangle {self.spring_engagement_angle}, scroll offset {self.scroll_offset}, total {self._value + self.scroll_offset}")
        return self._value + self.scroll_offset
    
    value = property(fget = get_yaw_value, fset = set_yaw_value)


class Joystick():
    
    def __init__(self, path_x, path_y, scaling_factor):
        self.subscriber_x = rospy.Subscriber(path_x, Intensity, self.msg_reciever_x)
        self.subscriber_y = rospy.Subscriber(path_y, Intensity, self.msg_reciever_y)
        self.x_value = None
        self.y_value = None
        self.raw_x_value = None
        self.raw_y_value = None
        self.scaling_factor = scaling_factor
        self.x_offset = 1600
        self.y_offset = 1600

        rospy.sleep(0.4)
        self.calibrate()
        
    def msg_reciever_x(self, msg):
        raw_x_value = msg.value
        x_value = (raw_x_value - self.x_offset) * self.scaling_factor
        if abs(x_value) < 0.1:
            x_value = 0
        
        self.raw_x_value = raw_x_value
        self.x_value = x_value
        

        
    def msg_reciever_y(self, msg):
        self.raw_y_value = msg.value
        self.y_value = (self.raw_y_value - self.y_offset) * self.scaling_factor
        if abs(self.y_value) < 0.1:
            self.y_value = 0
    
    def calibrate(self):
        if self.x_value is None or self.y_value is None:
            raise ValueError("no encoder data available to calibrate on")
        self.x_offset = self.raw_x_value
        self.y_offset = self.raw_y_value

    
class Switch():
    
    def __init__(self, pin_number: str):
        self.service = rospy.ServiceProxy('mirte_remote/mirte/get_pin_value', GetPinValue, persistent = True)
        self.pin_number = pin_number
        
    value = property(fget = lambda self: self.service(self.pin_number, "digital"))

class Joint():
    def __init__(self, path):
        self.subscriber = rospy.Subscriber(path, ServoPosition, self.msg_reciever_states)
        self.data = []
        self.timestamps = []
        
    def msg_reciever_states(self, msg): 
        self.timestamps.append(time.time() - t0)
        self.data.append(msg.angle)
        


        
        
        
        