import matplotlib.pyplot as plt

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

local_position = read_data("analysis/logs/min_snap_05_vehicle_local_position_0.csv")
set_point_position = read_data("analysis/logs/min_snap_05_vehicle_local_position_setpoint_0.csv")
# set_point = read_data("analysis/target_position.csv")

fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["x"],color='orange',label="position_x")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["x"],color='red',label="position_x reference")

ax.legend(handles=[l1,l2])
plt.grid()


fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["y"],color='orange',label="position_y")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["y"],color='red',label="position_y reference")

ax.legend(handles=[l1,l2])
plt.grid()


fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["z"],color='orange',label="position_z")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["z"],color='red',label="position_z reference")

ax.legend(handles=[l1,l2])
plt.grid()

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("3D position") # again passes data from all are being overwritten by passed data
ax.scatter3D(local_position["x"], local_position["y"], local_position["z"] )
ax.scatter3D(set_point_position["x"], set_point_position["y"], set_point_position["z"], 'red')
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ') 
plt.show()