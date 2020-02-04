# Import libraries
import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *

# Forms the starting values (degrees)
theta_1 = 0  
theta_2 = 90  
theta_3 = -90
theta_4 = 0
theta_5 = -90
theta_6 = 0
theta = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]) #numpy array
#joint_limits = [2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]
theta_max = 10 #Max delta theta value allowed

#Initial Variables
joints = 6
DOF_in = 3
pos_delta = np.array([0.0, 0.0, 0.0])


#Function to update theta values using scaled delta_theta 
def update_theta(theta_delta_raw):
    global theta
    global theta_max
    global pos_delta

    # Limit change in theta values using proportional weights (alpha)
    max_delta = np.amax(theta_delta_raw)
    alpha = 1
    new_theta = [0.0,0.0,0.0,0.0,0.0,0.0]
    theta_delta = [0.0,0.0,0.0,0.0,0.0,0.0]

    if max_delta > theta_max:
        alpha = 1/(float(max_delta/theta_max))
        #print(max_delta)
        #print(alpha)
    for i in range(len(theta_delta_raw)):
        theta_delta[i] = theta_delta_raw[i]*alpha
        new_theta[i] = theta[i]+theta_delta[i]
    
    #Assign updated theta values
    theta = new_theta 

    return theta_delta



def solve():
    import plotter
    global joints
    global theta
    global pos_delta
    pos_delta = plotter.pos_delta
    
    #Calculate Frames for given theta (convert to rad for trig functions)
    frames = model.find_frames(np.deg2rad(theta))[0:7]
    
    # Setting up the position vectors
    members = len(frames)
    X = np.zeros(members+1)
    Y = np.zeros(members+1)
    Z = np.zeros(members+1)
    for i in range(members):
        X[i+1] = frames[i][0, 3]
        Y[i+1] = frames[i][1, 3]
        Z[i+1] = frames[i][2, 3]
    
    plotter.plot(X, Y, Z)


    # Solve IK utilising the pseudo inverse jacobian method
    theta_delta_raw = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta, joints, DOF_in)
    #Return raw theta_delta values before weights
    #print(theta_delta)
    
    
    # Update the angles of each joint, uses division to further slow down the changei
    # theta_delta = plotter.pos_delta
    theta_delta = update_theta(theta_delta_raw)
    #print(theta_delta_raw[2])
    #print(theta_delta[2])
    time.sleep(0.01)
    return theta_delta


def main():
    while True:
        solve()


if __name__ == "__main__":
    main()