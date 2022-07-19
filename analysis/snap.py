from cProfile import label
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

df = read_data("trajectories/snap.csv")
set_way_points = read_data("trajectories/trajectory.csv")

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

plt.show()

fig3 = plt.figure(constrained_layout=True)
spec2 = gridspec.GridSpec(ncols=1, nrows=3, figure=fig3)
f2_ax2 = fig3.add_subplot(spec2[0, 0])
# f2_ax2.set_xlim([0,80])
f2_ax4 = fig3.add_subplot(spec2[1, 0])
# f2_ax4.set_xlim([0,80])
f2_ax6 = fig3.add_subplot(spec2[2, 0])
# f2_ax6.set_xlim([0,80])
# f2_ax8 = fig3.add_subplot(spec2[3, 0])
# f2_ax8.set_xlim([0,18])
# f2_ax9 = f2_ax4.twinx()