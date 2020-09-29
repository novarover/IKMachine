#Import libraries
import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *


#Define the starting values of theta (degrees)
theta_1 = 30 
theta_2 = 90  
theta_3 = -90
theta_4 = 0
theta_5 = -90
theta_6 = 0

#Define global variables
theta = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]) 
pos_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
pos_adapt = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#Max delta theta value allowed (deg/s)
#Prevents large angle changes at workspace limit
theta_max = 15  #Testing Variable

#Define joint limits for each angle [-180,180]
#Unlimited joints given 360 degree to avoid INF value on asymptote and allow limit rollover
qmin=[-90,-1000,-1000,-1000,-1000,-1000] #Testing Variable
qmax=[90,1000,1000,1000,1000,1000]

#Define Toggles (1 for On, 0 for Off)
#Toggle for clamping feature if joint limit is exceeded
clamping = 0
#Toggle for adaptive ref frame
adpt_frame = 1
#Toggle for joint limits
joint_limit = 0


#Function to clamp joint if it exceeds joint limit by 2 degrees
def clamp(theta):
    #H = Identity Matrix if no clamping
    global qmin
    global qmax

    limit = 2  #Testing Variable
    dimensions = len(theta)
    H = np.zeros((dimensions, dimensions))
    
    for i in range(dimensions):
        if (theta[i]>=qmax[i]+limit)|(theta[i]<=qmin[i]-limit):
            H[i,i]=0
            print('Clamped')    
        else:  
            H[i,i]=1
    return H


#Function to wrap any angle to[-180,180]
def wrapTo180(angle):
    angle = angle % 360
    angle = (angle + 360) % 360

    if (angle > 180):
        angle = angle - 360

    return angle


#Function to update theta values using scaled delta_theta 
def update_theta(theta_delta_raw):
    global theta
    global theta_max

    #Limit change in theta values using proportional weights (alpha)
    max_delta = np.amax(abs(theta_delta_raw))
    alpha = 1
    new_theta = [0.0,0.0,0.0,0.0,0.0,0.0]
    theta_delta = [0.0,0.0,0.0,0.0,0.0,0.0]
    
    
    #Check for joint limit breach and clamp if required
    if clamping == 1:
        H = clamp(theta)
        theta_delta_raw = np.dot(H,theta_delta_raw)

    #Compare max delta_theta value to max allowed value (deg/s)
    if max_delta > theta_max: 
        alpha = 1/(float(max_delta/theta_max))
    
    for i in range(len(theta_delta_raw)):
        theta_delta[i] = theta_delta_raw[i]*alpha #Scale by alpha
        new_theta[i] = theta[i]+theta_delta[i]    #Update angle values
        new_theta[i] = wrapTo180(new_theta[i])    #Wrap new angles to [-180,180]
    
    #Assign updated theta values (if sim)
    theta = new_theta 

    return theta_delta


#Looping function for model simulation
def solve():
    import plotter
    global theta
    global pos_delta
    
    #Read velocities from buttons
    pos_delta = plotter.pos_delta

    #Adaptive reference frame option
    if adpt_frame == 1:
        pos_adapt[0:3] = model.xyz_reframe(pos_delta[0:3], theta_1)  #Read intial base rotation angle to pass into reframe function
    else:
        pos_adapt[0:3] = pos_delta[0:3]


    #Calculate Frames for given theta (convert to rad for trig functions)
    frames = model.find_frames(np.deg2rad(theta))[0:7]
    
    #Setting up the position vectors
    members = len(frames)
    X = np.zeros(members+1)
    Y = np.zeros(members+1)
    Z = np.zeros(members+1)
    for i in range(members):
        X[i+1] = frames[i][0, 3]
        Y[i+1] = frames[i][1, 3]
        Z[i+1] = frames[i][2, 3]
    
    #Plot XYZ coordinates
    plotter.plot(X, Y, Z)

    #Return joint limit weights for given theta value
    if joint_limit == 1:
        W = weights.find_weights(theta)
    else:
        W = np.identity(6)

    #Solve IK utilising the pseudo inverse jacobian method
    #Return theta_delta values after applying weights to joints
    theta_delta = jacobian.pseudo_inverse(frames, pos_adapt, W)
    
    #Update the angles of each joint, uses weighting to reduce delta theta values to limit
    theta_delta = update_theta(theta_delta)

    #time.sleep(0.01)
    #print(W[0,0])
    return theta_delta


#Loop in simulation or implementation
def main():

    while True:
        solve()



if __name__ == "__main__":
    main()