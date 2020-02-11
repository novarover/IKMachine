#Import libraries
import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *

#Forms the starting values (degrees)
theta_1 = 0  
theta_2 = 90  
theta_3 = -90
theta_4 = 0
theta_5 = -90
theta_6 = 0

theta = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]) #numpy array
theta_max = 5 #Max delta theta value allowed (deg/s)

#Initial Variables
joints = 6
pos_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def clamp(theta):
    dimensions = len(theta)
    H = np.zeros((dimensions, dimensions))
    qmin=[-90,-30,-180,-180,-180,-180]
    qmax=[90,120,180,180,180,180]

    for i in range(dimensions):
        if (theta[i]>=qmax[i])|(theta[i]<=qmin[i]):
            H[i,i]=0
        else:  
            H[i,i]=1
    return H


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
   
    H = clamp(theta)
    theta_delta_raw = np.dot(H,theta_delta_raw)

    if max_delta > theta_max:
        alpha = 1/(float(max_delta/theta_max))
    
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
    #Return raw theta_delta values before weights
    theta_delta_raw = jacobian.pseudo_inverse(frames, X, Y, Z, pos_delta, joints)
    

    # Update the angles of each joint, uses weighting to reduce delta theta values to limit
    theta_delta = update_theta(theta_delta_raw)

    time.sleep(0.01)
    
    return theta_delta


def main():
    while True:
        solve()


if __name__ == "__main__":
    main()