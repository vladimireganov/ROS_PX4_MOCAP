import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import numpy as np

def read_data(file):
    return pd.read_csv(file,sep=",")

# path = "analysis/logs/square/square5" #square_calibrated5
# path = "analysis/logs/square/square2"
# path = "analysis\logs\min_snap_05"
# path = "/Volumes/GAMEZ/px4_data/flight_logs/log_0_2022-6-17-05-56-16" # weird flight data
# path = "/Volumes/GAMEZ/px4_data/flight_logs/log_43_2022-6-14-13-40-30"
# log_43_2022-6-14-13-40-30
# path = "E:/px4_data/flight_logs/log_43_2022-6-14-13-40-30"
# path = "E:/px4_data/flight_logs/log_0_2022-6-17-05-56-16"

# path = "E:/px4_data/faster_logging/log/2022-06-20/17_47_08/17_47_08" # weird flight

path = "E:/px4_data/faster_logging/log/2022-08-18/19_26_25/19_26_25"


# path = "E:/px4_data/weird_behavior/18_23_19/18_23_19" #crashed
# path = "E:/px4_data/weird_behavior/18_19_52/18_19_52"
# "E:\px4_data\faster_logging\log\2022-06-20\17_47_08.ulg"

local_position = read_data(path +"_vehicle_local_position_0.csv")
set_point_position = read_data(path +"_vehicle_local_position_setpoint_0.csv")
estimator_position = read_data(path +"_estimator_local_position_0.csv")
estimator_position2 = read_data(path +"_estimator_local_position_1.csv")
visual_position = read_data(path +"_estimator_visual_odometry_aligned_0.csv")

gyro = read_data(path +"_sensor_gyro_fifo_0.csv")

motion_cap = "MOCAP "
local_pos = "estimator "
sp = "set point"
# set_way_points = read_data("large_scale_traj_optimizer-main/example1/src/trajectory.csv")
# set_point = read_data("analysis/target_position.csv")

# print(local_position["timestamp"][0],set_point_position["timestamp"][0],estimator_position["timestamp"][0],visual_position["timestamp"][0])

estimator_position["timestamp"] = estimator_position["timestamp"] - set_point_position["timestamp"][0]
estimator_position2["timestamp"] = estimator_position2["timestamp"] - set_point_position["timestamp"][0]
visual_position["timestamp"] = visual_position["timestamp"] - set_point_position["timestamp"][0]
local_position["timestamp"] = local_position["timestamp"] - set_point_position["timestamp"][0]
gyro["timestamp"] = gyro["timestamp"] - set_point_position["timestamp"][0]
set_point_position["timestamp"] = set_point_position["timestamp"] - set_point_position["timestamp"][0]


local_position["timestamp"] = local_position["timestamp"] / 1e6
set_point_position["timestamp"] = set_point_position["timestamp"] / 1e6
estimator_position["timestamp"] = estimator_position["timestamp"] / 1e6
estimator_position2["timestamp"] = estimator_position2["timestamp"] / 1e6
visual_position["timestamp"] = visual_position["timestamp"] / 1e6
gyro["timestamp"] = gyro["timestamp"] / 1e6

local_position["dt"] = local_position["timestamp"].diff()
set_point_position["dt"] = set_point_position["timestamp"].diff()
estimator_position["dt"] = estimator_position["timestamp"].diff()
estimator_position2["dt"] = estimator_position2["timestamp"].diff()
visual_position["dt"] = visual_position["timestamp"].diff()
gyro["dt"] = gyro["timestamp"].diff()

local_position["freq"] = 1 / local_position["dt"]
set_point_position["freq"] = 1 / set_point_position["dt"]
estimator_position["freq"] = 1 / estimator_position["dt"]
estimator_position2["freq"] = 1 / estimator_position2["dt"]
visual_position["freq"] = 1 / visual_position["dt"]
gyro["freq"] = 1 / gyro["dt"]
# print(local_position["timestamp"][0],set_point_position["timestamp"][0],estimator_position["timestamp"][0],visual_position["timestamp"][0])

set_point_position["x_error"] = set_point_position["x"] - local_position["x"]
set_point_position["y_error"] = set_point_position["y"] - local_position["y"]
set_point_position["z_error"] = set_point_position["z"] - local_position["z"]


visual_position["x_error_sp"] = set_point_position["x"] - visual_position["x"]
visual_position["y_error_sp"] = set_point_position["y"] - visual_position["y"]
visual_position["z_error_sp"] = set_point_position["z"] - visual_position["z"]

visual_position["x_error_mocap"] = local_position["x"] - visual_position["x"]
visual_position["y_error_mocap"] = local_position["y"] - visual_position["y"]
visual_position["z_error_mocap"] = local_position["z"] - visual_position["z"]

local_position["yaw"] = local_position["heading"] / np.pi * 180
set_point_position["yaw"] = set_point_position["yaw"] / np.pi * 180

print(local_position["freq"].describe())
print(set_point_position["freq"].describe())
print(visual_position["freq"].describe())
#  Position X
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["x"],color='orange',label=local_pos + "position_x")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["x"],color='red',label="position_x reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["x"],color='blue',label= motion_cap + "x")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["x"],color='green',label="estimator_position_x")
# l5, = ax.plot(estimator_position2["timestamp"],estimator_position2["x"],color='green',label="estimator_position_x 2")

ax.legend(handles=[l1,l2,l3])
plt.grid()

#  Velocity x
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Velocity X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vx"],color='orange',label= local_pos + "velocity x")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vx"],color='red',label="velocity_x reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["vx"],color='green',label="estimator_velocity_x")

ax.legend(handles=[l1,l2])
plt.grid()


#  Position Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["y"],color='orange',label=local_pos +"position_y")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["y"],color='red',label="position_y reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["y"],color='blue',label= motion_cap + "y")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["y"],color='green',label="estimator_position_y")

ax.legend(handles=[l1,l2,l3])
plt.grid()


#  Velocity Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Velocity Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vy"],color='orange',label=local_pos+"velocity_y")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vy"],color='red',label="velocity_y reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["vy"],color='green',label="estimator_velocity_y")

ax.legend(handles=[l1,l2])
plt.grid()


#  Position Z
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["z"],color='orange',label=local_pos+"position_z")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["z"],color='red',label="position_z reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["z"],color='blue',label= motion_cap + "z")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["z"],color='green',label="estimator_position_z")

ax.legend(handles=[l1,l2,l3])
plt.grid()


# Velocity Z
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Velocity Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vz"],color='orange',label="velocity_z")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vz"],color='red',label="velocity_z reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
# l4, = ax.plot(estimator_position["timestamp"],estimator_position["vz"],color='green',label="estimator_velocity_z")

ax.legend(handles=[l1,l2])
plt.grid()


# Position X over Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('Position Y')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["y"],local_position["x"],color='orange',label=local_pos + "position")
l2, = ax.plot(set_point_position["y"],set_point_position["x"],color='red',label="position reference")
# l3 = ax.scatter(set_way_points["y"],set_way_points["x"],color='red',label="waypoints")
# ax.legend(handles=[l1,l2,l3])
l3, = ax.plot(visual_position["y"],visual_position["x"],color='blue',label= motion_cap )
# l4, = ax.plot(estimator_position["y"],estimator_position["x"],color='green',label="estimator_position")
# ax.legend(handles=[l1,l2,l3,l4])
ax.legend(handles=[l1,l2,l3])
plt.grid()




#  Heading 
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Heading from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("heading")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["yaw"],color='orange',label=local_pos+"heading")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["yaw"],color='red',label="heading reference")

ax.legend(handles=[l1,l2])
plt.grid()




fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("3D position estimated") # again passes data from all are being overwritten by passed data
ax.scatter3D(local_position["x"], local_position["y"], local_position["z"] )
ax.scatter3D(set_point_position["x"], set_point_position["y"], set_point_position["z"], 'red')
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ') 

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("3D position visual") # again passes data from all are being overwritten by passed data
ax.scatter3D(visual_position["x"], visual_position["y"], visual_position["z"] )
ax.scatter3D(set_point_position["x"], set_point_position["y"], set_point_position["z"], 'red')
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ') 

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("3D position visual and estimated") # again passes data from all are being overwritten by passed data
ax.scatter3D(visual_position["x"], visual_position["y"], visual_position["z"] )
ax.scatter3D(set_point_position["x"], set_point_position["y"], set_point_position["z"], 'red')
ax.scatter3D(local_position["x"], local_position["y"], local_position["z"] )
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ') 
# plt.show()


fig3 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=3, nrows=3, figure=fig3)
f2_ax3 = fig3.add_subplot(spec2[0, 0])
f2_ax4 = fig3.add_subplot(spec2[0, 1])
f2_ax5 = fig3.add_subplot(spec2[0, 2])

f2_ax0 = fig3.add_subplot(spec2[2, 0])
f2_ax1 = fig3.add_subplot(spec2[2, 1])
f2_ax2 = fig3.add_subplot(spec2[2, 2])

f2_ax6 = fig3.add_subplot(spec2[1, 0])
f2_ax7 = fig3.add_subplot(spec2[1, 1])
f2_ax8 = fig3.add_subplot(spec2[1, 2])
# delta_xy[0],delta_xy[1],delta_z
# ax,ay,az
# acceleration[0],acceleration[1],acceleration[2]
f2_ax0.set_xlabel('time (s)')
f2_ax0.set_ylabel("acceleration X")
f2_ax0.plot(local_position["timestamp"],local_position["ax"],color='blue')
f2_ax0.plot(set_point_position["timestamp"],set_point_position["acceleration[0]"],'r--',label="acceleration_x reference")

f2_ax1.set_xlabel('time (s)')
f2_ax1.set_ylabel("acceleration Y")
f2_ax1.plot(local_position["timestamp"],local_position["ay"],color='blue')
f2_ax1.plot(set_point_position["timestamp"],set_point_position["acceleration[1]"],'r--',label="acceleration_y reference")

f2_ax2.set_xlabel('time (s)')
f2_ax2.set_ylabel("acceleration Z")
f2_ax2.plot(local_position["timestamp"],local_position["az"],color='blue')
f2_ax2.plot(set_point_position["timestamp"],set_point_position["acceleration[2]"],'r--',label="acceleration_z reference")

f2_ax3.set_xlabel('time (s)')
f2_ax3.set_ylabel("X")
f2_ax3.plot(local_position["timestamp"],local_position["x"],color='blue')
f2_ax3.plot(set_point_position["timestamp"],set_point_position["x"],'r--',label="position_x reference")

f2_ax4.set_xlabel('time (s)')
f2_ax4.set_ylabel("Y")
f2_ax4.plot(local_position["timestamp"],local_position["y"],color='blue')
f2_ax4.plot(set_point_position["timestamp"],set_point_position["y"],'r--',label="position_y reference")

f2_ax5.set_xlabel('time (s)')
f2_ax5.set_ylabel("Z")
f2_ax5.plot(local_position["timestamp"],local_position["z"],color='blue')
f2_ax5.plot(set_point_position["timestamp"],set_point_position["z"],'r--',label="position_z reference")

# vx,vy,vz
f2_ax6.set_xlabel('time (s)')
f2_ax6.set_ylabel("velocity X")
f2_ax6.plot(local_position["timestamp"],local_position["vx"],color='blue')
f2_ax6.plot(set_point_position["timestamp"],set_point_position["vx"],'r--',label="velocity_x reference")

f2_ax7.set_xlabel('time (s)')
f2_ax7.set_ylabel("velocity Y")
f2_ax7.plot(local_position["timestamp"],local_position["vy"],color='blue')
f2_ax7.plot(set_point_position["timestamp"],set_point_position["vy"],'r--',label="velocity_y reference")

f2_ax8.set_xlabel('time (s)')
f2_ax8.set_ylabel("velocity Z")
f2_ax8.plot(local_position["timestamp"],local_position["vz"],color='blue')
f2_ax8.plot(set_point_position["timestamp"],set_point_position["vz"],'r--',label="velocity_z reference")

plt.grid()





# Position X error
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position error from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(set_point_position["timestamp"],set_point_position["x_error"],color='orange',label="error between esimator and setpoint")
l2, = ax.plot(visual_position["timestamp"],visual_position["x_error_sp"],color='green',label="error between mocap and setpoint")
l3, = ax.plot(visual_position["timestamp"],visual_position["x_error_mocap"],color='blue',label="error between mocap and local position")
ax.legend(handles=[l1,l2,l3])
plt.grid()

# Position Y error
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position error from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(set_point_position["timestamp"],set_point_position["y_error"],color='orange',label="error between esimator and setpoint")
l2, = ax.plot(visual_position["timestamp"],visual_position["y_error_sp"],color='green',label="error between mocap and setpoint")
l3, = ax.plot(visual_position["timestamp"],visual_position["y_error_mocap"],color='blue',label="error between mocap and local position")
ax.legend(handles=[l1,l2,l3])
plt.grid()




fig4 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig4)
f4_ax0 = fig4.add_subplot(spec2[0])
plt.grid()
f4_ax1 = fig4.add_subplot(spec2[1])
plt.grid()
f4_ax2 = fig4.add_subplot(spec2[2])
plt.grid()


# l1, = f4_ax0.plot(local_position["timestamp"],local_position["heading"],color='orange',label=local_pos + "heading")
l1, = f4_ax0.plot(local_position["timestamp"],local_position["yaw"],color='blue',label=local_pos + "heading")
l2, = f4_ax0.plot(set_point_position["timestamp"],set_point_position["yaw"],'r--',label=sp + "heading")
f4_ax0.set_ylabel("Heading")
f4_ax0.set_xlabel('time (s)')

f4_ax1.set_xlabel('time (s)')
f4_ax1.set_ylabel("X")
f4_ax1.plot(local_position["timestamp"],local_position["x"],color='blue')
f4_ax1.plot(set_point_position["timestamp"],set_point_position["x"],'r--',label="position_x reference")

f4_ax2.set_xlabel('time (s)')
f4_ax2.set_ylabel("Y")
f4_ax2.plot(local_position["timestamp"],local_position["y"],color='blue')
f4_ax2.plot(set_point_position["timestamp"],set_point_position["y"],'r--',label="position_y reference")

fig4.suptitle('Heading and X,Y positions')




fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["x"],color='orange',label=local_pos+"position_x")
l2, = ax.plot(local_position["timestamp"],local_position["y"],color='red',label=local_pos+"position_y")
l3, = ax.plot(local_position["timestamp"],local_position["z"],color='blue',label=local_pos+"position_z")

# l4, = ax.plot(estimator_position["timestamp"],estimator_position["x"],color='green',label="estimator_position_x")
# l5, = ax.plot(estimator_position2["timestamp"],estimator_position2["x"],color='green',label="estimator_position_x 2")

ax.legend(handles=[l1,l2,l3])
ax.set_xlim(5, 27)
plt.grid()


# frequency of writing into file
fig = plt.figure()
ax = plt.subplot()
ax.set_title("write frequency for px4")
ax.set_xlabel('time (s)')
ax.set_ylabel("frequency")
# ax.set_xlim([limL,limR])
l1, = ax.plot(set_point_position["timestamp"],set_point_position["freq"],color='blue',label="sp freq")
l2, = ax.plot(local_position["timestamp"],local_position["freq"],color='green',label=local_pos + "freq")
l3, = ax.plot(visual_position["timestamp"],visual_position["freq"],color='orange',label= motion_cap + "freq")
l4, = ax.plot(gyro["timestamp"],gyro["freq"],color='red',label= "gyro " + "freq")
ax.legend(handles=[l1,l2,l3,l4])
plt.grid()






fig4 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig4)
f4_ax0 = fig4.add_subplot(spec2[0])
plt.grid()
f4_ax1 = fig4.add_subplot(spec2[1])
plt.grid()
f4_ax2 = fig4.add_subplot(spec2[2])
plt.grid()


l1, = f4_ax0.plot(local_position["timestamp"],local_position["z"],color='orange',label=local_pos + "heading")
f4_ax0.set_ylabel("z m")
f4_ax0.set_xlabel('time (s)')

f4_ax1.set_xlabel('time (s)')
f4_ax1.set_ylabel("z m/s")
f4_ax1.plot(local_position["timestamp"],local_position["vz"],color='blue')
f4_ax1.plot(set_point_position["timestamp"],set_point_position["vz"],'r--',label="velocity z reference")

f4_ax2.set_xlabel('time (s)')
f4_ax2.set_ylabel("z m/s^2")
f4_ax2.plot(local_position["timestamp"],local_position["az"],color='blue')
f4_ax2.plot(set_point_position["timestamp"],set_point_position["acceleration[2]"],'r--',label="accelerattion z reference")

fig4.suptitle('Z profile')


plt.show()
