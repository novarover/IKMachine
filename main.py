import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *

# Forms the starting values
theta_1 = 0  # 1
theta_2 = 1  # 1.3
theta_3 = -2  # 1.6
theta_4 = 1  # 2*pi-(theta_2+theta_3)
theta_5 = 0
theta_6 = 0
theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
raw_encoders = model.find_encoder(theta)
joint_limits = [2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]#[2*pi,pi/3,0,pi/3,pi/3,2*pi]
theta_max = 0.6

goal = np.array([3.5, 0, 0, 0.2, 0.0, 0.0])   # 0.2,0.4,0.0
joints = 6
pos_delta = np.array([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
speed = 0.1


def up(self):
    global pos_delta

    pos_delta[1] = 0
    pos_delta[2] = 0
    pos_delta[0] = speed


def down(self):
    global pos_delta
    pos_delta[1] = 0
    pos_delta[2] = 0
    pos_delta[0] = speed


def get_euler(rotation):
    c = sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = c < 1e-6
    if not singular:
        y = atan2(-rotation[2, 0], c)  # Pitch
        x = atan2(rotation[2, 1]/cos(y), rotation[2, 2]/cos(y))  # Roll
        z = atan2(rotation[1, 0]/cos(y), rotation[0, 0]/cos(y))  # Yaw
    else:
        x = atan2(-rotation[1, 2], rotation[1, 1])
        y = atan2(-rotation[2, 0], c)
        z = 0
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


def update_theta(theta_delta):
    global theta
    global theta_max
    global pos_delta
    global raw_encoders
    max_delta = np.amax(theta_delta)
    alpha = 1
    new_theta = [0.0,0.0,0.0,0.0,0.0,0.0]
    if max_delta > theta_max:
        alpha = 1/(float(max_delta/theta_max))
    for i in range(len(theta_delta)):
        new_theta[i] = theta[i]+theta_delta[i]*alpha
        #if new_theta[i] > joint_limits[i]:
         #   new_theta[i] = joint_limits[i]
    #print(new_theta)
    new_theta[4] = theta[4]+theta_delta[4]*2
    raw_encoders = model.find_encoder(new_theta) 
#        theta[i] = new_theta
    #print(raw_encoders)


def solve():

    global goal
    global joints
    global pos_delta
    import plotter
    pos_delta = plotter.pos_delta
    global raw_encoders
    global theta
    theta = model.find_theta(raw_encoders)
    frames = model.find_frames(theta)[0:6]
    rotation = model.find_frames(theta)[6]
    #print(frames)
    # Drawing end effector claw
    wrist_r = model.revolute_joint(0,0,1,0)
    claw_1_r = model.revolute_joint(0.5,0,0.7,0)
    claw_2_r = model.revolute_joint(-0.5,0,0.7,0)
    claw_3_r = model.revolute_joint(0,0,0.7,pi/2.0)

    wrist = np.matmul(frames[-1],wrist_r)
    claw_1 = np.matmul(wrist,claw_1_r)
    claw_2 = np.matmul(wrist,claw_2_r)
    claw_3 = np.matmul(wrist,claw_3_r)

    claw_positions = [np.array(wrist[[0,1,2],[3,3,3]])[0],np.array(claw_1[[0,1,2],[3,3,3]])[0],np.array(claw_2[[0,1,2],[3,3,3]])[0],np.array(claw_3[[0,1,2],[3,3,3]])[0]]

    # Setting up the position vectors
    members = len(frames)
    X = np.zeros(members+1)
    Y = np.zeros(members+1)
    Z = np.zeros(members+1)
    for i in range(members):
        X[i+1] = frames[i][0, 3]
        Y[i+1] = frames[i][1, 3]
        Z[i+1] = frames[i][2, 3]

    plotter.plot(X, Y, Z, goal,claw_positions)

    # Calculate an appropriate change in position to solve for and put in into a column vector
    velocities = get_velocities([X[-1], Y[-1], Z[-1]], goal, rotation)
    #pos_delta = np.array(velocities[0:joints])
    # print(pos_delta)
    # Solve IK utilising the pseudo inverse jacobian method
    theta_delta = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta, joints)
    print(theta_delta)
    
    #print(theta_delta)
    # Update the angles of each joint, uses division to further slow down the changei
   # theta_delta = plotter.pos_delta
    update_theta(theta_delta)

    time.sleep(0.01)


def main():
    while True:
        solve()


if __name__ == "__main__":
    main()
