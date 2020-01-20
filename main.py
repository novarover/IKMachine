# Import libraries
import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *

# Forms the starting values (radians)
theta_1 = 0  
theta_2 = 1  
theta_3 = -2 
theta_4 = 1 
theta_5 = 0
theta_6 = 0
theta = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]) #numpy array
#raw_encoders = model.find_encoder(theta)
#raw_encoders = theta
joint_limits = [2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]
theta_max = 0.6 #Max delta theta value allowed

goal = np.array([3.5, 0, 0, 0.2, 0.0, 0.0])  
joints = 6
pos_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
speed = 0.1


def update_theta(theta_delta):
    global theta
    global theta_max
    global pos_delta
    global raw_encoders

    # Limit change in theta values using proportional weights (alpha)
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
    #raw_encoders = model.find_encoder(new_theta) 
    theta = new_theta 
#        theta[i] = new_theta
    #print(raw_encoders)


def solve():
    import plotter
    global goal
    global joints
    global pos_delta
    pos_delta = plotter.pos_delta
    global raw_encoders
    global theta
    #theta = model.find_theta(raw_encoders)
    frames = model.find_frames(theta)[0:6]
    #rotation = model.find_frames(theta)[6]
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
    #velocities = get_velocities([X[-1], Y[-1], Z[-1]], goal, rotation)
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
