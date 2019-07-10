import numpy as np
import plotter
import jacobian
import time
import model
from math import *

# Forms the starting values
theta_1 = 1
theta_2 = -1
theta_3 = 1
theta_4 = 0
theta_5 = 0
theta_6 = 0
theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

goal = np.array([-2, 2, 2, 0, 0, 0])
joints = 6
pos_delta = np.array([0, 0, -1])


class Directions(object):
    speed = 10

    def up(self, event):
        global pos_delta
        pos_delta[1] = 0
        pos_delta[2] = 0
        pos_delta[0] = self.speed

    def down(self, event):
        global pos_delta
        pos_delta[1] = 0
        pos_delta[2] = 0
        pos_delta[0] = -self.speed


def get_euler(rotation):
    c = sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = c < 1e-6
    if not singular:
        y = atan2(-rotation[2, 0], c)
        x = atan2(rotation[2, 1]/cos(y), rotation[2, 2]/cos(y))
        z = atan2(rotation[1, 0]/cos(y), rotation[0, 0]/cos(y))
    else:
        x = atan2(-rotation[1, 2], rotation[1, 1])
        y = atan2(-rotation[2, 0], c)
        z = 0
    print(z)
    return np.array([x, y, z])


def get_velocities(pos, goal, rotation):
    # Use the goal position and current position to find
    # an appropriate delta position to solve for
    x_delta = goal[0]-pos[0]
    y_delta = goal[1]-pos[1]
    z_delta = goal[2]-pos[2]
    xy_dist = sqrt(x_delta**2+y_delta**2)
    xyz_dist = sqrt(xy_dist**2+z_delta**2)
    xy_theta = atan2(y_delta, x_delta)

    # Calculate change in position related to a straight line towards goal
    z_theta = atan2(z_delta, xy_dist)
    x_delta = cos(xy_theta)*xy_dist
    y_delta = sin(xy_theta)*xy_dist
    z_delta = sin(z_theta)*xyz_dist

    # Get euler angles from rotation matrix
    [x_euler, y_euler, z_euler] = get_euler(rotation)

    x_euler_delta = goal[3] - x_euler
    y_euler_delta = goal[4] - y_euler
    z_euler_delta = goal[5] - z_euler
    print(z_euler_delta)
    velocities = [x_delta, y_delta, z_delta, x_euler_delta,
                  y_euler_delta, z_euler_delta, xy_dist, xyz_dist, z_theta]
    return velocities


def get_velocities_DLS(pos, goal, rotation):
    # Use the goal position and current position to find
    # delta position using damped least squares method
    na = np.array([rotation[0, 0], rotation[1, 0], rotation[2, 0]])
    oa = np.array([rotation[0, 1], rotation[1, 1], rotation[2, 1]])
    aa = np.array([rotation[0, 2], rotation[1, 2], rotation[2, 2]])
    nd = np.array([1, 0, 0])
    od = np.array([0, 1, 0])
    ad = np.array([0, 0, 1])
    x_delta = np.dot(na, goal[0:3]-pos[0:3])
    y_delta = np.dot(oa, goal[0:3]-pos[0:3])
    z_delta = np.dot(aa, goal[0:3]-pos[0:3])
    x_euler_delta = (np.dot(aa, od) - np.dot(ad, oa))/2.0
    y_euler_delta = (np.dot(na, ad) - np.dot(nd, aa))/2.0
    z_euler_delta = (np.dot(oa, nd) - np.dot(od, na))/2.0

    velocities = [x_delta, y_delta, z_delta, x_euler_delta,
                  y_euler_delta, z_euler_delta]
    return velocities


def solve():

    global goal
    global joints
    global pos_delta

    global theta
    frames = model.find_frames(theta)
    # Setting up the position vectors
    members = len(frames)
    X = np.zeros(members+1)
    Y = np.zeros(members+1)
    Z = np.zeros(members+1)
    for i in range(members):
        X[i+1] = frames[i][0, 3]
        Y[i+1] = frames[i][1, 3]
        Z[i+1] = frames[i][2, 3]

    plotter.plot(X, Y, Z, goal)

    # pos_delta = np.array(
    #     [goal[0]-X[-1], goal[1]-Y[-1], goal[2] - Z[-1]]).transpose()

    # Calculate an appropriate change in position to solve for and put in into a column vector
    velocities = get_velocities_DLS([X[-1], Y[-1], Z[-1]], goal, frames[-1])
    pos_delta = np.array(velocities[0: 6]).transpose()

    # Solve IK utilising the pseudo inverse jacobian method
    theta_delta = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta, joints)
    # Update the angles of each joint, uses division to further slow down the change
    for i in range(len(theta_delta)):
        theta[i] = theta_delta[i]/1.0
    time.sleep(0.01)


def main():
    while True:
        solve()


if __name__ == "__main__":
    main()
