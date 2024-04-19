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

print(os.getcwd())
print("Hello from data_analysis.py!")
print("This script will read the bag file and plot the data")
#print("Select the trajectory to analyze:")
#print("1) Linear trajectory")
#print("2) Linear trajectory with spline in the z-axis")
#print("3) Spline trajectory in all axes")


# Load the bag file

tau_bag = rosbag.Bag(os.getcwd() + '/bag/simulazione_quadrati/tau.bag')
state_bag = rosbag.Bag(os.getcwd() +'/bag/simulazione_quadrati/state.bag')
des_state_bag = rosbag.Bag(os.getcwd() +'/bag/simulazione_quadrati/des_state.bag')
way_bag = rosbag.Bag(os.getcwd() +'/bag/simulazione_quadrati/waypoints.bag')


############################################################################
############################# EXTRACT THE DATA #############################
############################################################################

tau_u = []
tau_v = []
tau_w = []
tau_r = []
tau_t = []

state_t = []
state_x = []
state_y = []
state_z = []
state_phi = []
state_theta = []
state_psi = []
state_u = []
state_v = []
state_w = []
state_p = []
state_q = []
state_r = []

des_state_t = []
des_state_x = []
des_state_y = []
des_state_z = []
des_state_phi = []
des_state_theta = []
des_state_psi = []
des_state_x_dot = []
des_state_y_dot = []
des_state_z_dot = []
des_state_phi_dot = []
des_state_theta_dot = []
des_state_psi_dot = []

way_x_list = []
way_y_list = []
way_z_list = []
way_t_list = []

tau_data = tau_bag.read_messages(topics=['tau_topic'])
state_data = state_bag.read_messages(topics=['state_topic'])
des_state_data = des_state_bag.read_messages(topics=['desired_state_topic'])
way_data = way_bag.read_messages(topics=['waypoints_topic'])


for topic, msg, t in tau_data:
    time = t.to_sec()
    t_u = msg.data[0]
    t_v = msg.data[1]
    t_w = msg.data[2]
    t_r = msg.data[3]
    
    tau_t.append(time)
    tau_u.append(t_u)
    tau_v.append(t_v)
    tau_w.append(t_w)
    tau_r.append(t_r)

t0 = tau_t[0]

for i in range(len(tau_t)):
    tau_t[i] = tau_t[i] - t0

for topic, msg, t in state_data:
    time = t.to_sec()
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    phi = msg.data[3]*180/np.pi
    theta = msg.data[4]*180/np.pi
    psi = msg.data[5]*180/np.pi
    u = msg.data[6]
    v = msg.data[7]
    w = msg.data[8]
    p = msg.data[9]*180/np.pi
    q = msg.data[10]*180/np.pi
    r = msg.data[11]*180/np.pi
    
    state_t.append(time)
    state_x.append(x)
    state_y.append(y)
    state_z.append(z)
    state_phi.append(phi)
    state_theta.append(theta)
    state_psi.append(psi)
    state_u.append(u)
    state_v.append(v)
    state_w.append(w)
    state_p.append(p)
    state_q.append(q)
    state_r.append(r)

t0 = state_t[0]

for i in range(len(state_t)):
    state_t[i] = state_t[i] - t0
    
for topic, msg, t in des_state_data:
    time_d = t.to_sec()
    print(msg.data)
    x_d = msg.data[0]
    y_d = msg.data[1]
    z_d = msg.data[2]
    phi_d = msg.data[3]*180/np.pi
    theta_d = msg.data[4]*180/np.pi
    psi_d = msg.data[5]*180/np.pi
    x_dot_d = msg.data[6]
    y_dot_d = msg.data[7]
    z_dot_d = msg.data[8]
    phi_dot_d = msg.data[9]*180/np.pi
    theta_dot_d = msg.data[10]*180/np.pi
    psi_dot_d = msg.data[11]*180/np.pi
    
    des_state_t.append(time_d)
    des_state_x.append(x_d)
    des_state_y.append(y_d)
    des_state_z.append(z_d)
    des_state_phi.append(phi_d)
    des_state_theta.append(theta_d)
    des_state_psi.append(psi_d)
    des_state_x_dot.append(x_dot_d)
    des_state_y_dot.append(y_dot_d)
    des_state_z_dot.append(z_dot_d)
    des_state_phi_dot.append(phi_dot_d)
    des_state_theta_dot.append(theta_dot_d)
    des_state_psi_dot.append(psi_dot_d)

t0 = des_state_t[0]

for i in range(len(des_state_t)):
    des_state_t[i] = des_state_t[i] - t0
    
for topic, msg, t in way_data:
    time_way = t.to_sec()
    way_x = msg.data[0]
    way_y = msg.data[1]
    way_z = msg.data[2]
    
    way_t_list.append(time_way)
    way_x_list.append(way_x)
    way_y_list.append(way_y)
    way_z_list.append(way_z)

t0 = way_t_list[0]
for i in range(len(way_t_list)):
    way_t_list[i] = way_t_list[i] - t0


des_state_bag.close()
state_bag.close()
tau_bag.close()
way_bag.close()

    
##########################################################################
#############################PLOT THE DATA################################
##########################################################################


# Plot 3D trajectory (Backstepping)

ax1_3D = plt.axes(projection='3d')
ax1_3D.plot3D(state_x, state_y, state_z, 'blue', label='Trajectory3D')

plt.title('3D trajectory')
ax1_3D.set_xlabel('x [m]')
ax1_3D.set_ylabel('y [m]')
ax1_3D.set_zlabel('z [m]')

# Plot 2D trajectory (Backstepping)

fig = plt.figure()
ax1_2D = plt.axes()
ax1_2D.plot(state_x, state_y, 'blue', label='Trajectory2D')
plt.title('2D trajectory (x-y)')
ax1_2D.set_xlabel('x [m]')
ax1_2D.set_ylabel('y [m]')


# Plot global-frame coordinates (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Global frame positions and heading (Backstepping)')
axes[0].plot(state_t, state_x, label="x")
axes[1].plot(state_t, state_y, label="y")
axes[2].plot(state_t, state_z, label="z")
axes[3].plot(state_t, state_psi, label="psi")

axes[0].set_ylabel('x [m]')
axes[1].set_ylabel('y [m]')
axes[2].set_ylabel('z [m]')
axes[3].set_ylabel('psi [°]')
axes[3].set_xlabel('Time [s]')

# Plot velocities (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Body frame velocities (Backstepping)')
axes[0].plot(state_t, state_u, label="u")
axes[1].plot(state_t, state_v, label="v")
axes[2].plot(state_t, state_w, label="w")
axes[3].plot(state_t, state_r, label="r")

axes[0].set_ylabel('u [m/s]')
axes[1].set_ylabel('v [m/s]')
axes[2].set_ylabel('w [m/s]')
axes[3].set_ylabel('r [°/s]')
axes[3].set_xlabel('Time [s]')

# Plot torques (Backstepping)
fig, axes = plt.subplots(4)
fig.suptitle('Backstepping Control inputs')
axes[0].plot(tau_t, tau_u, label="u")
axes[1].plot(tau_t, tau_v, label="v")
axes[2].plot(tau_t, tau_w, label="w")
axes[3].plot(tau_t, tau_r, label="r")

axes[0].set_ylabel('tau_u [N]')
axes[1].set_ylabel('tau_v [N]')
axes[2].set_ylabel('tau_w [N]')
axes[3].set_ylabel('tau_r [N*m]')

dt_backstepping = 0.01

power_consumed = 0
for i in range(len(tau_t)):
    power_consumed += (tau_u[i]**2 + tau_v[i]**2 + tau_w[i]**2 + tau_r[i]**2)*dt_backstepping

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