"""
Project Name: Teleoperation interface of Mirte Master
File: controller_data_processor.py
Description: For the 'data of joint states' gathered by the controller during operation, the data can be visualized

Author: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk
Repository: https://github.com/QSame9/Teleoperation-for-Mirte-Master

This file is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. 
It was commissioned by the Cognitive Robotics department of the TU Delft.
"""

import pandas as pd
import matplotlib.pyplot as plt



def plot_joint_data():
    df = pd.read_csv('joint_data.csv')
    
    # Clamp values of "data_elbow_enc" to a minimum of -1.6
    df.loc[df["data_elbow_enc"] < -1.6, "data_elbow_enc"] = -1.6

    
    displayed_joints = {"data_yaw_enc":     	0,
                        "data_yaw_mirte":       0,
                        "data_shoulder_enc":    0,
                        "data_shoulder_mirte":  0,
                        "data_elbow_enc":       0,
                        "data_elbow_mirte":     0,
                        "data_wrist_enc" :      0,
                        "data_wrist_mirte":     0,
                        "data_gripper_enc":     1,
                        "data_gripper_mirte":   1}#,]
#                        "force_data":           0,
#                        "object_width":         1}
    
    keys = df.keys()
    df["data_gripper_enc"] = df["data_gripper_enc"]*-(1/4)+0.47
    df["data_yaw_enc"] = df["data_yaw_enc"]*-1
#    df["object_width"] = df["object_width"]*-10e-3 + 0.05
    for i, key in enumerate(keys):
        if not key in displayed_joints:
            continue
        print(key)
        if displayed_joints[key] == 1:
            plt.plot(df.iloc[:,[i - 1]],df.iloc[:,[i]], label = key)

    
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor Value')
    plt.title('joint data')
    plt.legend(loc='lower left', bbox_to_anchor=(0, 0))
    plt.grid(True)
    plt.tight_layout()
    plt.show()
    
    
plot_joint_data()
