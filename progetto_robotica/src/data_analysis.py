#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import pandas as pd
import rosbag
import os
import pathlib

dir = os.path.join(os.path.dirname(__file__), os.path.pardir)
os.chdir(dir)


print("Hello from data_analysis.py!")
print("This script will read the bag file and plot the data\n")
print("Select the trajectory to analyze:")
print("1) Linear trajectory")
print("2) Linear trajectory with spline in the z-axis")
print("3) Spline trajectory in all axes")

selection = input("Enter the number of the trajectory: ")

# Load the bag file

if selection == "1":
    backstepping_tau_bag = rosbag.Bag(os.getcwd() + '/bag/backstepping/linear/tau.bag')
    backstepping_state_bag = rosbag.Bag(os.getcwd() +'/bag/backstepping/linear/state.bag')
    MPC_tau_bag = rosbag.Bag(os.getcwd() +'/bag/MPC/linear/tau.bag')
    MPC_state_bag = rosbag.Bag(os.getcwd() +'/bag/MPC/linear/state.bag')

elif selection == "2":
    backstepping_tau_bag = rosbag.Bag(os.getcwd() + '/bag/backstepping/sinz/tau.bag')
    backstepping_state_bag = rosbag.Bag(os.getcwd() +'/bag/backstepping/sinz/state.bag')
    MPC_tau_bag = rosbag.Bag(os.getcwd() + '/bag/MPC/sinz/tau.bag')
    MPC_state_bag = rosbag.Bag(os.getcwd() + '/bag/MPC/sinz/state.bag')

else:
    backstepping_tau_bag = rosbag.Bag(os.getcwd() + '/bag/backstepping/sinxyz/tau.bag')
    backstepping_state_bag = rosbag.Bag(os.getcwd() + '/bag/backstepping/sinxyz/state.bag')
    MPC_tau_bag = rosbag.Bag(os.getcwd() + '/bag/MPC/sinxyz/tau.bag')
    MPC_state_bag = rosbag.Bag(os.getcwd() + '/bag/MPC/sinxyz/state.bag')


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

sphere1_x = 10.0
sphere1_y = 10.0
sphere1_z = 3.0
sphere2_x = -8.0
sphere2_y = -12.0
sphere2_z = 2.0
box1_x = 14.0
box1_y = 10.0
box1_z = 1.0
box2_x = 10.0
box2_y = -10.0
box2_z = 1.5


# Plot 3D trajectory (Backstepping)

ax1_3D = plt.axes(projection='3d')
ax1_3D.plot3D(backstepping_state_x, backstepping_state_y, backstepping_state_z, 'blue', label='Backstepping3D')
ax1_3D.scatter(sphere1_x, sphere1_y, sphere1_z, 'red', label='boa1')
ax1_3D.scatter(sphere2_x, sphere2_y, sphere2_z, 'red', label='boa2')
ax1_3D.scatter(box1_x, box1_y, box1_z, 'green', label='boa3')
ax1_3D.scatter(box2_x, box2_y, box2_z, 'green', label='boa4')

plt.title('Backstepping control trajectory')
ax1_3D.set_xlabel('x [m]')
ax1_3D.set_ylabel('y [m]')
ax1_3D.set_zlabel('z [m]')

# Plot 2D trajectory (Backstepping)

fig = plt.figure()
ax1_2D = plt.axes()
ax1_2D.plot(backstepping_state_x, backstepping_state_y, 'blue', label='Backstepping2D')
ax1_2D.scatter(sphere1_x, sphere1_y, edgecolors='red',linewidths=2, label='boa1')
ax1_2D.scatter(sphere2_x, sphere2_y,  edgecolors='red',linewidths=2, label='boa2')
ax1_2D.scatter(box1_x, box1_y,  edgecolors='green',linewidths=2, label='boa3')
ax1_2D.scatter(box2_x, box2_y,  edgecolors='green',linewidths=2, label='boa4')
plt.title('Backstepping control trajectory (x-y)')
ax1_2D.set_xlabel('x [m]')
ax1_2D.set_ylabel('y [m]')


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

dt_backstepping = 0.01

power_consumed = 0
for i in range(len(backstepping_tau_t)):
    power_consumed += (backstepping_tau_u[i]**2 + backstepping_tau_v[i]**2 + backstepping_tau_w[i]**2 + backstepping_tau_r[i]**2)*dt_backstepping

print("Backstepping power consumed: ", power_consumed)

plt.show()

# Plot 3D trajectory (MPC)

ax2_3D = plt.axes(projection='3d')
ax2_3D.plot3D(MPC_state_x, MPC_state_y, MPC_state_z, 'blue', label='MPC3D')
ax2_3D.scatter(sphere1_x, sphere1_y, sphere1_z, 'red', label='boa1')
ax2_3D.scatter(sphere2_x, sphere2_y, sphere2_z, 'red', label='boa2')
ax2_3D.scatter(box1_x, box1_y, box1_z, 'green', label='boa3')
ax2_3D.scatter(box2_x, box2_y, box2_z, 'green', label='boa4')

plt.title('MPC control trajectory')
ax2_3D.set_xlabel('x [m]')
ax2_3D.set_ylabel('y [m]')
ax2_3D.set_zlabel('z [m]')

# Plot 2D trajectory (MPC)

fig = plt.figure()
ax2_2D = plt.axes()
ax2_2D.plot(MPC_state_x, MPC_state_y, 'blue', label='MPC2D')
ax2_2D.scatter(sphere1_x, sphere1_y, edgecolors='red',linewidths=2, label='boa1')
ax2_2D.scatter(sphere2_x, sphere2_y,  edgecolors='red',linewidths=2, label='boa2')
ax2_2D.scatter(box1_x, box1_y,  edgecolors='green',linewidths=2, label='boa3')
ax2_2D.scatter(box2_x, box2_y,  edgecolors='green',linewidths=2, label='boa4')
plt.title('Backstepping control trajectory (x-y)')
ax2_2D.set_xlabel('x [m]')
ax2_2D.set_ylabel('y [m]')

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

dt_MPC = 0.01

power_consumed_MPC = 0
for i in range(len(MPC_tau_t)):
    power_consumed_MPC += (MPC_tau_u[i]**2 + MPC_tau_v[i]**2 + MPC_tau_w[i]**2 + MPC_tau_r[i]**2)*dt_MPC

print("MPC power consumed: ", power_consumed_MPC)

plt.show()
