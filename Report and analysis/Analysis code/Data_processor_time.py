"""
Project Name: Teleoperation interface of Mirte Master
File: Data_processor_time.py
Description: For the 'data of time needed per control executable' gathered by the controller during operation, the data can be visualized

Author: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk
Repository: https://github.com/QSame9/Teleoperation-for-Mirte-Master

This file is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. 
It was commissioned by the Cognitive Robotics department of the TU Delft.
"""

import pandas as pd
import matplotlib.pyplot as plt

def plot_time_data():
    df = pd.read_csv("time_step_data (1).csv")
    df.plot()
    plt.xlabel('loop nr.')
    plt.ylabel('time')
    plt.title('time step data')
    plt.ylim(0, 0.5)
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    # plt.tight_layout()
    plt.show()
    
plot_time_data()
