from cProfile import label
import matplotlib.pyplot as plt

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

df = read_data("large_scale_traj_optimizer-main/example1/src/snap.csv")

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
l1, = ax.plot(df["position_y"],df["position_x"],color='orange',label="position_x")
ax.legend(handles=[l1])
plt.grid()

plt.show()