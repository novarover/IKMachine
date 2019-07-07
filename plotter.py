import pickle
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib
matplotlib.use("Qt5agg")
matplotlib.rc('axes.formatter', useoffset=False)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
fig.show()


def plot(X, Y, Z, goal):

    ax.cla()
    ax.plot(X, Y, Z)
    ax.plot([X[-1]], [Y[-1]], [Z[-1]], marker='o', markersize=3, color="red")
    ax.plot([X[0]], [Y[0]], [Z[0]], marker='o', markersize=3, color="blue")
    ax.plot([goal[0]], [goal[1]], [goal[2]],
            marker='+', markersize=3, color="blue")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    fig.canvas.draw_idle()
    fig.canvas.start_event_loop(0.001)
