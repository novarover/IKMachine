import numpy as np
import plotter
import jacobian
import time
from math import *

theta_1 = 1
theta_2 = 1.3
theta_3 = 0.6
theta_4 = 2*pi-(theta_2+theta_3)
goal = np.array([4, 3, 1])
joints = 3


def DHframe(theta, d, a, alpha):
    frame_array = np.mat([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                          [sin(theta), cos(theta)*cos(alpha),
                           -cos(theta)*sin(alpha), a*sin(theta)],
                          [0, sin(alpha), cos(alpha), d],
                          [0, 0, 0, 1]])
    return frame_array


def get_velocities(pos, goal):
    x_delta = goal[0]-pos[0]
    y_delta = goal[1]-pos[1]
    z_delta = goal[2]-pos[2]
    xy_dist = sqrt(x_delta**2+y_delta**2)
    xyz_dist = sqrt(xy_dist**2+z_delta**2)
    xy_theta = atan2(y_delta, x_delta)
    z_theta = atan2(z_delta, xy_dist)
    x_delta = cos(xy_theta)*xy_dist
    y_delta = sin(xy_theta)*xy_dist
    z_delta = sin(z_theta)*xyz_dist
    velocities = [x_delta, y_delta, z_delta, xy_dist, xyz_dist, z_theta]
    return velocities


def model():
    global theta_1
    global theta_2
    global theta_3
    global theta_4
    global goal
    global joints

    frame_01 = DHframe(theta_1, 2, 0, pi/2)
    frame_12 = DHframe(theta_2, 0, 4, 0)
    frame_23 = DHframe(theta_3, 0, 3, 0)
    # frame_34 = DHframe(theta_4, 0, 2, 0)
    frame_02 = np.matmul(frame_01, frame_12)
    frame_03 = np.matmul(frame_02, frame_23)
    # frame_04 = np.matmul(frame_03, frame_34)
    frames = [frame_01, frame_02, frame_03]
    joints = len(frames)
    X = np.zeros(joints+1)
    Y = np.zeros(joints+1)
    Z = np.zeros(joints+1)
    for i in range(joints):
        X[i+1] = frames[i][0, 3]
        Y[i+1] = frames[i][1, 3]
        Z[i+1] = frames[i][2, 3]
    plotter.plot(X, Y, Z, goal)
    # pos_delta = np.array(
    #     [goal[0]-X[-1], goal[1]-Y[-1], goal[2] - Z[-1]]).transpose()
    velocities = get_velocities([X[-1], Y[-1], Z[-1]], goal)
    pos_delta = np.array(velocities[0:3]).transpose()
    theta_delta = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta)
    theta_1 = theta_1+theta_delta[0]/10.0
    theta_2 = theta_2+theta_delta[1]/10.0
    theta_3 = theta_3+theta_delta[2]/10.0

    time.sleep(0.01)


while True:
    model()
