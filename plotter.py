import pickle
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.widgets import Button
import main
import matplotlib
pos_delta = np.array([0.0, 0.0, 0.0])
speed = 1 


def xp(self):
    pos_delta[0] = pos_delta[0] + speed
    

def xn(self):
    pos_delta[0] = pos_delta[0] - speed


def yp(self):
    pos_delta[1] = pos_delta[1] + speed


def yn(self):
    pos_delta[1] = pos_delta[1] - speed


def zp(self):
    pos_delta[2] = pos_delta[2] + speed


def zn(self):
    pos_delta[2] = pos_delta[2] - speed


# Required for updating without bringing the window to the front
matplotlib.use("Qt5agg")
matplotlib.rc('axes.formatter', useoffset=False)

# Set up plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
fig.show()

# Set up buttons
ax_xp = plt.axes([0.81, 0.2, 0.1, 0.075])
button_xp = Button(ax_xp, 'X+')
button_xp.on_clicked(xp)
ax_xn = plt.axes([0.7, 0.2, 0.1, 0.075])
button_xn = Button(ax_xn, 'X-')
button_xn.on_clicked(xn)
ax_yp = plt.axes([0.81, 0.1, 0.1, 0.075])
button_yp = Button(ax_yp, 'Y+')
button_yp.on_clicked(yp)
ax_yn = plt.axes([0.7, 0.1, 0.1, 0.075])
button_yn = Button(ax_yn, 'Y-')
button_yn.on_clicked(yn)
ax_zp = plt.axes([0.81, 0.01, 0.1, 0.075])
button_zp = Button(ax_zp, 'Z+')
button_zp.on_clicked(zp)
ax_zn = plt.axes([0.7, 0.01, 0.1, 0.075])
button_zn = Button(ax_zn, 'Z-')
button_zn.on_clicked(zn)


def plot(X, Y, Z):
    # Clear plot each time its called and replot the arms
    ax.cla()
    ax.plot(X, Y, Z)

    ax.plot([X[0]], [Y[0]], [Z[0]], marker='o',
            markersize=3, color="blue")  # Plots the origin
    ax.plot([X[-1]], [Y[-1]], [Z[-1]], marker='o', markersize=3,
            color="red")  # Plots the end effector position
    ax.plot([X[-2]], [Y[-2]], [Z[-2]], marker='o', markersize=3,
    color="blue")  # Plots the wrist position
   
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.set_zlim(-100, 100)

    # Draw without bringing window to front
    fig.canvas.draw_idle()
    fig.canvas.start_event_loop(0.001)