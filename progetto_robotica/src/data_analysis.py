#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import pandas as pd
import rosbag

print("Hello from data_analysis.py!")
print("This script will read the bag file and plot the data")
print("Select the trajectory to analyze:")
print("1) Linear trajectory")
print("2) Linear trajectory with spline in the z-axis")
print("3) Spline trajectory in all axes")

selection = input("Enter the number of the trajectory: ")

# Load the bag file

if selection == "1":
    backstepping_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/linear/backstepping_tau_linear.bag')
    backstepping_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/linear/state_backstepping_linear.bag')
    MPC_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/linear/MPC_tau_linear.bag')
    MPC_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/linear/state_MPC_linear.bag')

elif selection == "2":
    backstepping_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/sinz/backstepping_tau_sinz.bag')
    backstepping_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/sinz/state_backstepping_sinz.bag')
    MPC_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/sinz/MPC_tau_sinz.bag')
    MPC_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/sinz/state_MPC_sinz.bag')

else:
    backstepping_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/sinxyz/backstepping_tau_sinxyz.bag')
    backstepping_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/backstepping test/sinxyz/state_backstepping_sinxyz.bag')
    MPC_tau_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/sinxyz/MPC_tau_sinxyz.bag')
    MPC_state_bag = rosbag.Bag('/home/antonio/catkin_ws/src/progetto_robotica/bag/MPC test/sinxyz/state_MPC_sinxyz.bag')


# Read the bag file

backstepping_tau_data = backstepping_tau_bag.read_messages(topics=['tau_topic'])
backstepping_state_data = backstepping_state_bag.read_messages(topics=['state_topic'])
MPC_tau_data = MPC_tau_bag.read_messages(topics=['tau_topic'])
MPC_state_data = MPC_state_bag.read_messages(topics=['state_topic'])

############################################################################
############################# EXTRACT THE DATA #############################
############################################################################

# Backstepping torques data
backstepping_tau_t = []
backstepping_tau_u = []
backstepping_tau_v = []
backstepping_tau_w = []
backstepping_tau_r = []

for topic, msg, t in backstepping_tau_data:
    time = t.to_sec()
    tau_u = msg.data[0]
    tau_v = msg.data[1]
    tau_w = msg.data[2]
    tau_r = msg.data[3]
    
    backstepping_tau_t.append(time)
    backstepping_tau_u.append(tau_u)
    backstepping_tau_v.append(tau_v)
    backstepping_tau_w.append(tau_w)
    backstepping_tau_r.append(tau_r)

t0 = backstepping_tau_t[0]

for i in range(len(backstepping_tau_t)):
    backstepping_tau_t[i] = backstepping_tau_t[i] - t0



# Backstepping state data
backstepping_state_t = []
backstepping_state_x = []
backstepping_state_y = []
backstepping_state_z = []
backstepping_state_psi = []
backstepping_state_u = []
backstepping_state_v = []
backstepping_state_w = []
backstepping_state_r = []

for topic, msg, t in backstepping_state_data:
    time = t.to_sec()
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    psi = msg.data[3]*180/np.pi
    u = msg.data[4]
    v = msg.data[5]
    w = msg.data[6]
    r = msg.data[7]*180/np.pi
    
    backstepping_state_t.append(time)
    backstepping_state_x.append(x)
    backstepping_state_y.append(y)
    backstepping_state_z.append(z)
    backstepping_state_psi.append(psi)
    backstepping_state_u.append(u)
    backstepping_state_v.append(v)
    backstepping_state_w.append(w)
    backstepping_state_r.append(r)
    

t0 = backstepping_state_t[0]

for i in range(len(backstepping_state_t)):
    backstepping_state_t[i] = backstepping_state_t[i] - t0

backstepping_state_bag.close()
backstepping_tau_bag.close()

# MPC torques data
MPC_tau_t = []
MPC_tau_u = []
MPC_tau_v = []
MPC_tau_w = []
MPC_tau_r = []

for topic, msg, t in MPC_tau_data:
    time = t.to_sec()
    tau_u = msg.data[0]
    tau_v = msg.data[1]
    tau_w = msg.data[2]
    tau_r = msg.data[3]
    
    MPC_tau_t.append(time)
    MPC_tau_u.append(tau_u)
    MPC_tau_v.append(tau_v)
    MPC_tau_w.append(tau_w)
    MPC_tau_r.append(tau_r)
    
    
t0 = MPC_tau_t[0]

for i in range(len(MPC_tau_t)):
    MPC_tau_t[i] = MPC_tau_t[i] - t0

# MPC state data
MPC_state_t = []
MPC_state_x = []
MPC_state_y = []
MPC_state_z = []
MPC_state_psi = []
MPC_state_u = []
MPC_state_v = []
MPC_state_w = []
MPC_state_r = []

for topic, msg, t in MPC_state_data:
    time = t.to_sec()
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    psi = msg.data[3]*180/np.pi
    u = msg.data[4]
    v = msg.data[5]
    w = msg.data[6]
    r = msg.data[7]*180/np.pi
    
    MPC_state_t.append(time)
    MPC_state_x.append(x)
    MPC_state_y.append(y)
    MPC_state_z.append(z)
    MPC_state_psi.append(psi)
    MPC_state_u.append(u)
    MPC_state_v.append(v)
    MPC_state_w.append(w)
    MPC_state_r.append(r)
    
    
t0 = MPC_state_t[0]

for i in range(len(MPC_state_t)):
    MPC_state_t[i] = MPC_state_t[i] - t0
    
MPC_state_bag.close()
MPC_tau_bag.close()
    
##########################################################################
#############################PLOT THE DATA################################
##########################################################################

# Plot 3D trajectory (Backstepping)

ax1 = plt.axes(projection='3d')
ax1.plot3D(backstepping_state_x, backstepping_state_y, backstepping_state_z, 'blue', label='Backstepping')
plt.title('Backstepping control trajectory')
ax1.set_xlabel('x [m]')
ax1.set_ylabel('y [m]')
ax1.set_zlabel('z [m]')


# Plot global-frame coordinates (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Global frame positions and heading (Backstepping)')
axes[0].plot(backstepping_state_t, backstepping_state_x, label="x")
axes[1].plot(backstepping_state_t, backstepping_state_y, label="y")
axes[2].plot(backstepping_state_t, backstepping_state_z, label="z")
axes[3].plot(backstepping_state_t, backstepping_state_psi, label="psi")

axes[0].set_ylabel('x [m]')
axes[1].set_ylabel('y [m]')
axes[2].set_ylabel('z [m]')
axes[3].set_ylabel('psi [°]')
axes[3].set_xlabel('Time [s]')

# Plot velocities (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Body frame velocities (Backstepping)')
axes[0].plot(backstepping_state_t, backstepping_state_u, label="u")
axes[1].plot(backstepping_state_t, backstepping_state_v, label="v")
axes[2].plot(backstepping_state_t, backstepping_state_w, label="w")
axes[3].plot(backstepping_state_t, backstepping_state_r, label="r")

axes[0].set_ylabel('u [m/s]')
axes[1].set_ylabel('v [m/s]')
axes[2].set_ylabel('w [m/s]')
axes[3].set_ylabel('r [°/s]')
axes[3].set_xlabel('Time [s]')

# Plot torques (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Backstepping Control inputs')
axes[0].plot(backstepping_tau_t, backstepping_tau_u, label="u")
axes[1].plot(backstepping_tau_t, backstepping_tau_v, label="v")
axes[2].plot(backstepping_tau_t, backstepping_tau_w, label="w")
axes[3].plot(backstepping_tau_t, backstepping_tau_r, label="r")

axes[0].set_ylabel('tau_u [N]')
axes[1].set_ylabel('tau_v [N]')
axes[2].set_ylabel('tau_w [N]')
axes[3].set_ylabel('tau_r [N*m]')

dt_backstepping = 0.001

power_consumed = 0
for i in range(len(backstepping_tau_t)):
    power_consumed += (backstepping_tau_u[i]**2 + backstepping_tau_v[i]**2 + backstepping_tau_w[i]**2 + backstepping_tau_r[i]**2)*dt_backstepping

print("Backstepping power consumed: ", power_consumed)

plt.show()

# Plot 3D trajectory (MPC)

ax2 = plt.axes(projection='3d')
ax2.plot3D(MPC_state_x, MPC_state_y, MPC_state_z, 'red', label='MPC')
plt.title('MPC control trajectory')
ax2.set_xlabel('x [m]')
ax2.set_ylabel('y [m]')
ax2.set_zlabel('z [m]')


# Plot global-frame coordinates (MPC)
fig, axes = plt.subplots(4)
fig.suptitle('Global frame positions and heading (MPC)')
axes[0].plot(MPC_state_t, MPC_state_x, label="x")
axes[1].plot(MPC_state_t, MPC_state_y, label="y")
axes[2].plot(MPC_state_t, MPC_state_z, label="z")
axes[3].plot(MPC_state_t, MPC_state_psi, label="psi")

axes[0].set_ylabel('x [m]')
axes[1].set_ylabel('y [m]')
axes[2].set_ylabel('z [m]')
axes[3].set_ylabel('psi [°]')
axes[3].set_xlabel('Time [s]')

# Plot velocities (MPC)
fig, axes = plt.subplots(4)
fig.suptitle('Body frame velocities (MPC)')
axes[0].plot(MPC_state_t, MPC_state_u, label="u")
axes[1].plot(MPC_state_t, MPC_state_v, label="v")
axes[2].plot(MPC_state_t, MPC_state_w, label="w")
axes[3].plot(MPC_state_t, MPC_state_r, label="r")

axes[0].set_ylabel('u [m/s]')
axes[1].set_ylabel('v [m/s]')
axes[2].set_ylabel('w [m/s]')
axes[3].set_ylabel('r [°/s]')
axes[3].set_xlabel('Time [s]')

# Plot torques (MPC)
fig, axes = plt.subplots(4)
fig.suptitle('MPC Control inputs')
axes[0].plot(MPC_tau_t, MPC_tau_u, label="u")
axes[1].plot(MPC_tau_t, MPC_tau_v, label="v")
axes[2].plot(MPC_tau_t, MPC_tau_w, label="w")
axes[3].plot(MPC_tau_t, MPC_tau_r, label="r")

axes[0].set_ylabel('tau_u [N]')
axes[1].set_ylabel('tau_v [N]')
axes[2].set_ylabel('tau_w [N]')
axes[3].set_ylabel('tau_r [N*m]')

dt_MPC = 0.1

power_consumed_MPC = 0
for i in range(len(MPC_tau_t)):
    power_consumed_MPC += (MPC_tau_u[i]**2 + MPC_tau_v[i]**2 + MPC_tau_w[i]**2 + MPC_tau_r[i]**2)*dt_MPC

print("MPC power consumed: ", power_consumed_MPC)

plt.show()