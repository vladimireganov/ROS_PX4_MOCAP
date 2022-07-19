import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

local_position = read_data("analysis/logs/min_snap_05_vehicle_local_position_0.csv")
set_point_position = read_data("analysis/logs/min_snap_05_vehicle_local_position_setpoint_0.csv")
estimator_position = read_data("analysis/logs/min_snap_05_estimator_local_position_0.csv")
estimator_position2 = read_data("analysis/logs/min_snap_05_estimator_local_position_1.csv")
visual_position = read_data("analysis/logs/min_snap_05_estimator_visual_odometry_aligned_0.csv")
set_way_points = read_data("large_scale_traj_optimizer-main/example1/src/trajectory.csv")
# set_point = read_data("analysis/target_position.csv")


#  Position X
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["x"],color='orange',label="position_x")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["x"],color='red',label="position_x reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["x"],color='blue',label="visual_position_x")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["x"],color='green',label="estimator_position_x")

ax.legend(handles=[l1,l2,l3,l4])
plt.grid()

#  Velocity x
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Velocity X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vx"],color='orange',label="velocity_x")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vx"],color='red',label="velocity_x reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["vx"],color='green',label="estimator_velocity_x")

ax.legend(handles=[l1,l2,l4])
plt.grid()


#  Position Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["y"],color='orange',label="position_y")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["y"],color='red',label="position_y reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["y"],color='blue',label="visual_position_y")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["y"],color='green',label="estimator_position_y")

ax.legend(handles=[l1,l2,l3,l4])
plt.grid()


#  Velocity Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Velocity Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vy"],color='orange',label="velocity_y")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vy"],color='red',label="velocity_y reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["vy"],color='green',label="estimator_velocity_y")

ax.legend(handles=[l1,l2,l4])
plt.grid()


#  Position Z
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["z"],color='orange',label="position_z")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["z"],color='red',label="position_z reference")
l3, = ax.plot(visual_position["timestamp"],visual_position["z"],color='blue',label="visual_position_z")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["z"],color='green',label="estimator_position_z")

ax.legend(handles=[l1,l2,l3,l4])
plt.grid()


# Velocity Z
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Velocity from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Velocity Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["vz"],color='orange',label="velocity_z")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["vz"],color='red',label="velocity_z reference")
# l3, = ax.plot(visual_position["timestamp"],visual_position["vx"],color='blue',label="visual_velociy_x")
l4, = ax.plot(estimator_position["timestamp"],estimator_position["vz"],color='green',label="estimator_velocity_z")

ax.legend(handles=[l1,l2,l4])
plt.grid()


# Position X over Y
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('Position Y')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["y"],local_position["x"],color='orange',label="position")
l2, = ax.plot(set_point_position["y"],set_point_position["x"],color='red',label="position reference")
# l3 = ax.scatter(set_way_points["y"],set_way_points["x"],color='red',label="waypoints")
# ax.legend(handles=[l1,l2,l3])
l3, = ax.plot(visual_position["y"],visual_position["x"],color='blue',label="visual_position")
l4, = ax.plot(estimator_position["y"],estimator_position["x"],color='green',label="estimator_position")
ax.legend(handles=[l1,l2,l3,l4])
plt.grid()




#  Heading 
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Heading from px4")
ax.set_xlabel('timestamp')
ax.set_ylabel("heading")
# ax.set_xlim([limL,limR])
l1, = ax.plot(local_position["timestamp"],local_position["heading"],color='orange',label="heading")
l2, = ax.plot(set_point_position["timestamp"],set_point_position["yaw"],color='red',label="heading reference")

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
f2_ax0.set_xlabel('time stamp')
f2_ax0.set_ylabel("acceleration X")
f2_ax0.plot(local_position["timestamp"],local_position["ax"],color='blue')
f2_ax0.plot(set_point_position["timestamp"],set_point_position["acceleration[0]"],'r--',label="acceleration_x reference")

f2_ax1.set_xlabel('time stamp')
f2_ax1.set_ylabel("acceleration Y")
f2_ax1.plot(local_position["timestamp"],local_position["ay"],color='blue')
f2_ax1.plot(set_point_position["timestamp"],set_point_position["acceleration[1]"],'r--',label="acceleration_y reference")

f2_ax2.set_xlabel('time stamp')
f2_ax2.set_ylabel("acceleration Z")
f2_ax2.plot(local_position["timestamp"],local_position["az"],color='blue')
f2_ax2.plot(set_point_position["timestamp"],set_point_position["acceleration[2]"],'r--',label="acceleration_z reference")

f2_ax3.set_xlabel('time stamp')
f2_ax3.set_ylabel("X")
f2_ax3.plot(local_position["timestamp"],local_position["x"],color='blue')
f2_ax3.plot(set_point_position["timestamp"],set_point_position["x"],'r--',label="position_x reference")

f2_ax4.set_xlabel('time stamp')
f2_ax4.set_ylabel("Y")
f2_ax4.plot(local_position["timestamp"],local_position["y"],color='blue')
f2_ax4.plot(set_point_position["timestamp"],set_point_position["y"],'r--',label="position_y reference")

f2_ax5.set_xlabel('time stamp')
f2_ax5.set_ylabel("Z")
f2_ax5.plot(local_position["timestamp"],local_position["z"],color='blue')
f2_ax5.plot(set_point_position["timestamp"],set_point_position["z"],'r--',label="position_z reference")

# vx,vy,vz
f2_ax6.set_xlabel('time stamp')
f2_ax6.set_ylabel("velocity X")
f2_ax6.plot(local_position["timestamp"],local_position["vx"],color='blue')
f2_ax6.plot(set_point_position["timestamp"],set_point_position["vx"],'r--',label="velocity_x reference")

f2_ax7.set_xlabel('time stamp')
f2_ax7.set_ylabel("velocity Y")
f2_ax7.plot(local_position["timestamp"],local_position["vy"],color='blue')
f2_ax7.plot(set_point_position["timestamp"],set_point_position["vy"],'r--',label="velocity_y reference")

f2_ax8.set_xlabel('time stamp')
f2_ax8.set_ylabel("velocity Z")
f2_ax8.plot(local_position["timestamp"],local_position["vz"],color='blue')
f2_ax8.plot(set_point_position["timestamp"],set_point_position["vz"],'r--',label="velocity_z reference")

plt.grid()
plt.show()