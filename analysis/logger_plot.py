import matplotlib.pyplot as plt

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

postion = read_data("analysis/position.csv")
state = read_data("analysis/state.csv")
set_point = read_data("analysis/target_position.csv")

fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position X")
# ax.set_xlim([limL,limR])
l1, = ax.plot(postion["stamp"],postion["position_x"],color='orange',label="position_x")
ax.legend(handles=[l1])
plt.grid()

fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Y")
# ax.set_xlim([limL,limR])
l1, = ax.plot(postion["stamp"],postion["position_y"],color='orange',label="position_y")
ax.legend(handles=[l1])
plt.grid()

fig = plt.figure()
ax = plt.subplot()
ax.set_title("Position from px4")
ax.set_xlabel('stamp')
ax.set_ylabel("Position Z")
# ax.set_xlim([limL,limR])
l1, = ax.plot(postion["stamp"],postion["position_z"],color='orange',label="position_z")
ax.legend(handles=[l1])
plt.grid()

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title("3D position") # again passes data from all are being overwritten by passed data
ax.scatter3D(postion["position_x"], postion["position_y"], postion["position_z"], 'gray')


plt.show()