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

tau_data = tau_bag.read_messages(topics=['tau_topic'])
state_data = state_bag.read_messages(topics=['state_topic'])
des_state_data = des_state_bag.read_messages(topics=['desired_state_topic'])

way = [0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, -2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 4.0, 0.0, 2.0, 4.0, 2.0, 2.0, 4.0, 2.0, -2.0, 4.0, 0.0, -2.0, 4.0, 0.0, 0.0, 4.0]
for i in range(int(len(way)/3)):
    way_x_list.append(way[i*3])
    way_y_list.append(way[i*3+1])
    way_z_list.append(way[i*3+2])

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
    
    
state_bag.close()
tau_bag.close()
des_state_bag.close()
    
##########################################################################
#############################PLOT THE DATA################################
##########################################################################


# Plot 3D trajectory 
way_z_list_to_plot = []
state_z_to_plot = []
des_state_z_to_plot = []

for i in range(len(way_z_list)):
    way_z_list_to_plot.append(-way_z_list[i])
    
for i in range(len(state_z)):
    state_z_to_plot.append(-state_z[i])

for i in range(len(des_state_z)):
    des_state_z_to_plot.append(-des_state_z[i])

target = [way_x_list[-1], way_y_list[-1], way_z_list_to_plot[-1]]
start = [state_x[0], state_y[0], state_z_to_plot[0]]

ax1_3D = plt.axes(projection='3d')
ax1_3D.plot3D(state_y, state_x, state_z_to_plot, 'blue', label='Trajectory3D')
ax1_3D.plot(way_y_list, way_x_list, way_z_list_to_plot, color='red',marker = "o", markersize = 5,linestyle = "", label='waypoints')
ax1_3D.plot([start[1]], [start[0]], [start[2]], color='black',markerfacecolor = "green", marker = "*", markersize = 10,linestyle = "",label='start')
ax1_3D.plot([target[1]], [target[0]], [target[2]], color='black',markerfacecolor = "yellow", marker = "*", markersize = 10,linestyle = "",label='target')

plt.title('3D trajectory')
ax1_3D.set_xlabel('x [m]')
ax1_3D.set_ylabel('y [m]')
ax1_3D.set_zlabel('z [m]')
plt.legend()

fig2 = plt.figure()

ax2_3D = plt.axes(projection='3d')
ax2_3D.plot3D(des_state_y, des_state_x, des_state_z_to_plot, 'blue', label='Desired Trajectory3D')
ax2_3D.plot(way_y_list, way_x_list, way_z_list_to_plot, color='red',marker = "o", markersize = 5,linestyle = "",label='waypoints')
ax2_3D.plot([start[1]], [start[0]], [start[2]], color='black',markerfacecolor = "green", marker = "*", markersize = 10,linestyle = "",label='start')
ax2_3D.plot([target[1]], [target[0]], [target[2]], color='black',markerfacecolor = "yellow", marker = "*", markersize = 10,linestyle = "",label='target')

plt.title('3D desired trajectory')
ax2_3D.set_xlabel('x [m]')
ax2_3D.set_ylabel('y [m]')
ax2_3D.set_zlabel('z [m]')
plt.legend()

# Plot 2D trajectory 

fig = plt.figure()
ax1_2D = plt.axes()
ax1_2D.plot(state_y, state_x, 'blue', label='Trajectory2D')
ax1_2D.plot(way_y_list, way_x_list, color='red',marker = "o", markersize = 5,linestyle = "")

plt.title('2D trajectory (x-y)')
ax1_2D.set_xlabel('x [m]')
ax1_2D.set_ylabel('y [m]')


# Plot global-frame coordinates 
fig, axes = plt.subplots(4)
fig.suptitle('Global frame positions and heading')
axes[0].plot(state_t, state_x, label="x")
axes[1].plot(state_t, state_y, label="y")
axes[2].plot(state_t, state_z, label="z")
axes[3].plot(state_t, state_psi, label="psi")

axes[0].set_ylabel('x [m]')
axes[1].set_ylabel('y [m]')
axes[2].set_ylabel('z [m]')
axes[3].set_ylabel('psi [°]')
axes[3].set_xlabel('Time [s]')

# Plot velocities 
fig, axes = plt.subplots(4)
fig.suptitle('Body frame velocities')
axes[0].plot(state_t, state_u, label="u")
axes[1].plot(state_t, state_v, label="v")
axes[2].plot(state_t, state_w, label="w")
axes[3].plot(state_t, state_r, label="r")

axes[0].set_ylabel('u [m/s]')
axes[1].set_ylabel('v [m/s]')
axes[2].set_ylabel('w [m/s]')
axes[3].set_ylabel('r [°/s]')
axes[3].set_xlabel('Time [s]')

# Plot torques 
fig, axes = plt.subplots(4)
fig.suptitle('Control inputs')
axes[0].plot(tau_t, tau_u, label="u")
axes[1].plot(tau_t, tau_v, label="v")
axes[2].plot(tau_t, tau_w, label="w")
axes[3].plot(tau_t, tau_r, label="r")

axes[0].set_ylabel('tau_u [N]')
axes[1].set_ylabel('tau_v [N]')
axes[2].set_ylabel('tau_w [N]')
axes[3].set_ylabel('tau_r [N*m]')

dt_controller = 0.005

power_consumed = 0
for i in range(len(tau_t)):
    power_consumed += (tau_u[i]**2 + tau_v[i]**2 + tau_w[i]**2 + tau_r[i]**2)*dt_controller

print("power consumed: ", power_consumed)

plt.show()

