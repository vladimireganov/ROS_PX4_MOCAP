import matplotlib.pyplot as plt

import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

postion = read_data()
state = read_data()
set_point = read_data()

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