"""
Project Name: Controller code for physical Teleoperation interface of Mirte Master
File: gripper_simulator_file_final.py
Description: As the Mirte Master does not have force sensors, a simulator is used to 
define the force needed for force feedback

Author: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk
License: MIT
Repository: https://github.com/QSame9/Teleoperation-for-Mirte-Master

This file is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. 
It was commissioned by the Cognitive Robotics department of the TU Delft.


This file is distributed under the terms
of the MIT License. See the LICENSE file in the root of this repository
or visit https://opensource.org/licenses/MIT for more details.
"""

# import tkinter as tk
from math import pi
import time

object_width = 30 #mm
object_spring_constant = 2 #N/mm
max_force = 10 #Newton
scaling_factor = 3

class GripperSimulator:
    def __init__(self): #, parent, width=400, height=400):
        # self.canvas = tk.Canvas(parent, width=width, height=height, bg='white')
        # self.canvas.pack()

        # self.width = width
        # self.height = height

        # Initial gripper parameters
        self.gripper_distance = None  # Default finger gap
        self.commanded_opening_distance = None # Default finger gap
        # self.finger_y = self.height // 2  # Gripper and object center Y
        # self.finger_length = 50
        # self.text_color = "green"

        # Object dimensions
        
        # self.object_radius_x = object_width / 2  # width radius (squished horizontally)
        # self.object_radius_y = object_width / 2  # height radius (squished vertically)
        # self.draw_scene()
        
        self.exerted_force = 0

    # def draw_scene(self):
    #     self.canvas.delete("all")
    #     center_x = self.width // 2

    #     # Draw "squished" object at gripper height
    #     #print(f"drawing oval({self.object_radius_x, self.object_radius_y})")
    #     self.canvas.create_oval(
    #         center_x - self.object_radius_x * scaling_factor, self.finger_y - self.object_radius_y * scaling_factor,
    #         center_x + self.object_radius_x * scaling_factor, self.finger_y + self.object_radius_y * scaling_factor,
    #         fill='grey', tags='object'
    #     )
        
    #     half_gap = self.gripper_distance * scaling_factor / 2
    #     commanded_half_gap = self.commanded_opening_distance * scaling_factor / 2

    #     # Draw gripper fingers
    #     # Left finger
    #     self.canvas.create_line(
    #         center_x - half_gap - 5, self.finger_y - self.finger_length,
    #         center_x - half_gap - 5, self.finger_y + self.finger_length,
    #         width=10, fill="grey"
    #     )
        
    #     self.canvas.create_line(
    #         center_x - commanded_half_gap, self.finger_y - self.finger_length,
    #         center_x - commanded_half_gap, self.finger_y + self.finger_length,
    #         width=1, fill="red"
    #     )
        
    #     # Right finger
    #     self.canvas.create_line(
    #         center_x + half_gap + 5, self.finger_y - self.finger_length,
    #         center_x + half_gap + 5, self.finger_y + self.finger_length,
    #         width=10, fill="grey"
    #     )
        
    #     self.canvas.create_line(
    #         center_x + commanded_half_gap, self.finger_y - self.finger_length,
    #         center_x + commanded_half_gap, self.finger_y + self.finger_length,
    #         width=1, fill="red"
    #     )

    #     # Show gripped state if fingers are close enough to object
    #     if self.gripper_distance < object_width:
    #         self.canvas.create_text(center_x, self.finger_y + object_width * scaling_factor, text=f"force: {self.exerted_force} N", fill=self.text_color, font=('Arial', 12))

    def set_gripper_angle(self, angle: float):
        """Set the horizontal distance between gripper fingers.
        
        -angle(float): angle published to mirtes gripper -0.5 to 0.5
        """
        opening_distance = (angle + 0.5) * 120 # 0 to 50 mm
        self.commanded_opening_distance = opening_distance
        #print(f"opening_distance: {opening_distance}")
        self.exerted_force = round(max(0, (object_width - opening_distance) * object_spring_constant), 3)
        
        if self.exerted_force < max_force:
            # self.gripper_distance = opening_distance
            self.exerted_force = self.exerted_force
            # self.text_color = "green"
        else:
            self.exerted_force = max_force
            # self.text_color = "red"
            
        # if self.exerted_force > 0:
        #     self.object_radius_x = self.gripper_distance / 2
        #     self.object_radius_y = ((object_width / 2) ** 2) / self.object_radius_x
        # else:
        #     self.object_radius_x = object_width / 2
        #     self.object_radius_y = object_width / 2
            
        # self.draw_scene()
        
    def get_gripping_force(self):
        # time.sleep(0.001)
        #print(f"exerted_force {self.exerted_force}, opening_distance{self.commanded_opening_distance}, object_width {object_width}")
        return self.exerted_force
