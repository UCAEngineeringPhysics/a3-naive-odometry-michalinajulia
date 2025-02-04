"""
Run this script with local Python (on RPi).
"""

import sys
import os
import csv
import matplotlib.pyplot as plt
from math import pi, sin, cos

# Extract data
data_dir = os.path.join(sys.path[0], "data")
### START CODING HERE ### ~ 1 line
data_file = os.path.join(data_dir, "example_data.csv")  # use your own data
### END CODING HERE ###
with open(data_file, newline="") as f:
    reader = csv.reader(f)
    vel_data = tuple(reader)
real_vels = []
for vd in vel_data:
    real_vels.append((float(vd[0]), float(vd[1])))
# Create target velocities
ref_vels = (
    (0.5, 0.0),
    (0.5, pi / 8),
    (0.4, 0.0),
    (0.35, -pi / 4),
    (0.0, -2 * pi / 3),
    (-0.35, -pi / 4),
    (-0.4, 0.0),
    (-0.5, pi / 8),
    (-0.5, 0.0),
    (0.0, 2 * pi / 3),
)
targ_vels = []
for i in range(len(real_vels)):
    targ_vels.append(ref_vels[int(i / 40)])
print(len(targ_vels))

# Calculate trajectory
x, y, th = [0], [0], [0]
x_hat, y_hat, th_hat = [0], [0], [0]
dt = 0.05  # seconds
for i in range(len(targ_vels)):
    ### START CODING HERE ### ~ 6 lines
    # Compute ideal trajectory
    dx = None
    dy = None
    dth = None
    # Compute actual trajectory
    dx_hat = None
    dy_hat = None
    dth_hat = None
    ### END CODING HERE ###
    # Store ideal state
    x.append(x[-1] + dx)
    y.append(y[-1] + dy)
    th.append(th[-1] + dth)
    # Store actual state
    x_hat.append(x_hat[-1] + dx_hat)
    y_hat.append(y_hat[-1] + dy_hat)
    th_hat.append(th_hat[-1] + dth_hat)

# Plot data
fig, ax = plt.subplots(1, 2, figsize=(16, 8))
# Plot position trajectory
ax[0].scatter(x, y)
ax[0].scatter(x_hat, y_hat, marker="+")
ax[0].set_xlabel("X (m)")
ax[0].set_ylabel("Y (m)")
ax[0].set_xlim([-0.25, 3.25])
ax[0].set_ylim([-0.25, 3.25])
ax[0].grid()
ax[0].legend(["target", "actual"])
# Plot orientation traj
ts = list(range(len(x)))  # create timestamps for x axis
for i in range(len(x)):
    ts[i] = 0.05 * i
ax[1].plot(ts, th, ".", markersize="10")
ax[1].plot(ts, th_hat, "+", markersize="5")
ax[1].set_xlabel("Time (s)")
ax[1].set_ylabel("θ (radians)")
ax[1].set_xlim([-0.25, 20.25])
ax[1].set_ylim([-pi * 2.5, pi])
ax[1].grid()
ax[1].legend(["target", "actual"])
# Title
### CHOOSE APPROPRIATE TITLE ###
fig.suptitle("Trajectory Compare - Noload", fontsize=16)
plt.savefig("noload_traj.png")
# fig.suptitle("Trajectory Compare - Ground", fontsize=16)
# plt.savefig("ground_traj.png")
plt.show()
