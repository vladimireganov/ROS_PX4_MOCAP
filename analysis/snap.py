from cProfile import label
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

df = read_data("trajectories/8_figure_snap.csv")
set_way_points = read_data("trajectories/8_shape.csv")

fig = plt.figure()
ax = plt.subplot()
ax.set_title("Minimum snap trajectory generation")
ax.set_xlabel('Time (s)')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(df["time"],df["position_x"],color='orange',label="position_x")
ax.legend(handles=[l1])
plt.grid()

# plt.show()
fig = plt.figure()
ax = plt.subplot()
ax.set_title("Minimum snap trajectory generation")
ax.set_xlabel('Time (s)')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(df["time"],df["position_y"],color='orange',label="position_y")
ax.legend(handles=[l1])
plt.grid()


fig = plt.figure()
ax = plt.subplot()
ax.set_title("Minimum snap trajectory generation")
ax.set_xlabel('Position Y')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(df["position_y"],df["position_x"],color='orange',label="trajectory")
l2 = ax.scatter(set_way_points["y"],set_way_points["x"],color='red',label="waypoints")
ax.legend(handles=[l1,l2])
plt.grid()


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("Minimum snap trajectory generation") # again passes data from all are being overwritten by passed data
ax.scatter3D(df["position_x"], df["position_y"], df["position_z"], 'gray')
# l2, = ax.scatter(set_way_points["y"],set_way_points["x"],color='red',label="waypoints")
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ')



fig3 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig3)

f2_ax2 = fig3.add_subplot(spec2[0, 0])
l1, = f2_ax2.plot(df["time"],df["position_x"],color='orange',label="x position")
f2_ax2.legend(handles=[l1])
plt.grid()
# f2_ax2.set_xlim([0,80])
f2_ax4 = fig3.add_subplot(spec2[1, 0])
l1, = f2_ax4.plot(df["time"],df["velocity_x"],color='orange',label="x velocity")
f2_ax4.legend(handles=[l1])
plt.grid()
# f2_ax4.set_xlim([0,80])
f2_ax6 = fig3.add_subplot(spec2[2, 0])
l1, = f2_ax6.plot(df["time"],df["acceleration_x"],color='orange',label="x acceleration")
f2_ax6.legend(handles=[l1])
plt.grid()



fig3 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig3)

f2_ax2 = fig3.add_subplot(spec2[0, 0])
l1, = f2_ax2.plot(df["time"],df["position_y"],color='orange',label="y position")
f2_ax2.legend(handles=[l1])
plt.grid()
# f2_ax2.set_xlim([0,80])
f2_ax4 = fig3.add_subplot(spec2[1, 0])
l1, = f2_ax4.plot(df["time"],df["velocity_y"],color='orange',label="y velocity")
f2_ax4.legend(handles=[l1])
plt.grid()
# f2_ax4.set_xlim([0,80])
f2_ax6 = fig3.add_subplot(spec2[2, 0])
l1, = f2_ax6.plot(df["time"],df["acceleration_y"],color='orange',label="y acceleration")
f2_ax6.legend(handles=[l1])
plt.grid()




fig3 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig3)

f2_ax2 = fig3.add_subplot(spec2[0, 0])
l1, = f2_ax2.plot(df["time"],df["position_z"],color='orange',label="z position")
f2_ax2.legend(handles=[l1])
plt.grid()
# f2_ax2.set_xlim([0,80])
f2_ax4 = fig3.add_subplot(spec2[1, 0])
l1, = f2_ax4.plot(df["time"],df["velocity_z"],color='orange',label="z velocity")
f2_ax4.legend(handles=[l1])
plt.grid()
# f2_ax4.set_xlim([0,80])
f2_ax6 = fig3.add_subplot(spec2[2, 0])
l1, = f2_ax6.plot(df["time"],df["acceleration_z"],color='orange',label="z acceleration")
f2_ax6.legend(handles=[l1])
plt.grid()


plt.show()