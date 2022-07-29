import matplotlib.pyplot as plt

#  make as a function
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
# l5, = ax.plot(estimator_position2["timestamp"],estimator_position2["x"],color='green',label="estimator_position_x 2")

ax.legend(handles=[l1,l2,l3,l4])
plt.grid()