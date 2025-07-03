"""
Code written by ChatGPT, used prompt was:
I have the data in the excel sheet, and I want to know the delay
that happens per command. Can you write a script that:
- for every joint x "enc" data point, looks for a respective
    joint x "servo" data point that lies soonest after the time
    the enc data point was sent out. The "servo" data point should
    be within a certain tolerance of the enc data value.
- Make the tolerance easily changable.
- When finding this "quickest within range" servo data point,
    calculate the difference in time, aka delay, between when
    the servo reaches this value, and when the encoder sent the value.
- When the delay is more than 0.5 seconds, leave the value empty,
    so that in a graph, that part of the graph would be interrupted.
- Plot the delay per encoder command per joint
- Calculate the average delay per joint, and put that at the top right
    of the plot of the corresponding joint.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === Configuration ===
CSV_FILE = "joint_data.csv"  # Change path if needed
TOLERANCE = 0.1  # Acceptable value difference between encoder and servo
MAX_DELAY = 5   # Maximum allowed delay in seconds

# === Load data ===
df = pd.read_csv(CSV_FILE)
df.loc[df["data_elbow_enc"] < -1.6, "data_elbow_enc"] = -1.6
df["data_gripper_enc"] = df["data_gripper_enc"]*-(1/4)+0.47

# === Joints to analyze ===
joints = ["yaw", "shoulder", "elbow", "wrist", "gripper"]
delays_by_joint = {}
average_delays = {}
median_delays = {}

# === Delay Calculation ===
for joint in joints:
    t_enc = df[f"t_{joint}_enc"].values
    data_enc = df[f"data_{joint}_enc"].values
    t_mirte = df[f"t_{joint}_mirte"].values
    data_mirte = df[f"data_{joint}_mirte"].values

    delays = []
    for t, val in zip(t_enc, data_enc):
        mask = (t_mirte > t) & (np.abs(data_mirte - val) <= TOLERANCE)
        candidates = t_mirte[mask]

        if len(candidates) == 0:
            delays.append(np.nan)
        else:
            delay = candidates[0] - t
            delays.append(delay if delay <= MAX_DELAY else np.nan)

    delays_by_joint[joint] = np.array(delays)
    average_delays[joint] = np.nanmean(delays_by_joint[joint])
    median_delays[joint] = np.nanmedian(delays_by_joint[joint])
    print(f"{joint} median is {median_delays[joint]}")

# === Plotting ===
fig, axs = plt.subplots(len(joints), 1, figsize=(10, 12), sharex=True)

for idx, joint in enumerate(joints):
    axs[idx].plot(df[f"t_{joint}_enc"], delays_by_joint[joint], label=f"{joint} delay")
    axs[idx].axhline(MAX_DELAY, color='r', linestyle='--', alpha=0.3, label="Max allowed delay")
    axs[idx].set_ylabel("Delay (s)")
    axs[idx].legend(loc="upper left")
    avg_delay = average_delays[joint]
    med_delay = median_delays[joint]
    axs[idx].text(0.99, 0.95, f"Median delay: {med_delay:.3f}s \n Average delay: {avg_delay:.3f}s", transform=axs[idx].transAxes,
                  ha='right', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.6))

axs[-1].set_xlabel("Time (s)")
plt.suptitle("Delay per Encoder Command per Joint")
plt.tight_layout(rect=[0, 0.03, 1, 0.97])
plt.grid(True)
plt.show()
