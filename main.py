import numpy as np
import plotter
import jacobian
import time
import model
from math import *

# Forms the starting values
theta_1 = 1
theta_2 = 1.3
theta_3 = 0.6
theta_4 = 2*pi-(theta_2+theta_3)


goal = np.array([-3, 3, 4, 0.1, 0.2, 0.3])
joints = 3
pos_delta = np.array([0, 0, -1])


class Directions(object):
    speed = 10

    def up(self, event):
        global pos_delta
        print("Up")
        pos_delta[1] = 0
        pos_delta[2] = 0
        pos_delta[0] = self.speed

    def down(self, event):
        global pos_delta
        pos_delta[1] = 0
        pos_delta[2] = 0
        pos_delta[0] = -self.speed


def get_velocities(pos, goal):
    # Use the goal position and current position to find
    # an appropriate delta position to solve for
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


def solve():
    global theta_1
    global theta_2
    global theta_3
    global theta_4

    global goal
    global joints
    global pos_delta

    theta = [theta_1, theta_2, theta_3, theta_4]
    frames = model.find_frames(theta)

    # Setting up the position vectors
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

    # Calculate an appropriate change in position to solve for and put in into a column vector
    velocities = get_velocities([X[-1], Y[-1], Z[-1]], goal)
    pos_delta = np.array(velocities[0:3]).transpose()

    # Solve IK utilising the pseudo inverse jacobian method
    theta_delta = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta)
    # Update the angles of each joint, uses division to further slow down the change
    theta_1 = theta_1+theta_delta[0]/10.0
    theta_2 = theta_2+theta_delta[1]/10.0
    theta_3 = theta_3+theta_delta[2]/10.0
    time.sleep(0.01)


def main():
    while True:
        solve()


if __name__ == "__main__":
    main()
