#Import libraries
import numpy as np
import plotter
import jacobian
import time
import model
import weights
from math import *

#Toggle simulation (0 for rover implementation)
sim = 1

#Define the starting values of theta (degrees)
theta_1 = 0  
theta_2 = 90  
theta_3 = -90
theta_4 = 0
theta_5 = -90
theta_6 = 0

#Define global variables
if sim == 1:    
    theta = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]) 
    pos_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#Max delta theta value allowed (deg/s)
#Prevents large angle changes at workspace limit
theta_max = 10 

#Define joint limits for each angle [-180,180]
#Unlimited joints given 181 degree to avoid INF value on asymptote
qmin=[-90,-30,-181,-181,-181,-181]
qmax=[90,120,181,181,181,181]



#Function to read theta input from encoders
def input_theta():

    theta = np.array([0.0, 90.0, -90.0, 0.0, -90.0, 0.0])

    return theta


#Function to read velocity input from joysticks
def input_posdelta():

    pos_delta = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

    return pos_delta


#Function to output theta_delta command
def output_thetadelta(theta_delta):

    theta_delta_out = theta_delta

    return theta_delta_out



#Function to clamp joint if it exceeds joint limit by 2 degrees
def clamp(theta):
    #H = Identity Matrix if no clamping
    global qmin
    global qmax

    dimensions = len(theta)
    H = np.zeros((dimensions, dimensions))
    
    for i in range(dimensions):
        if (theta[i]>=qmax[i]+2)|(theta[i]<=qmin[i]-2):
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
def simulation():
    import plotter
    global theta
    global pos_delta
    
    #Read velocities from buttons
    pos_delta = plotter.pos_delta
    
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
    W = weights.find_weights(theta)

    #Solve IK utilising the pseudo inverse jacobian method
    #Return theta_delta values after applying weights to joints
    theta_delta = jacobian.pseudo_inverse(frames, pos_delta, W)
    
    #print(theta[0])
 
    #Update the angles of each joint, uses weighting to reduce delta theta values to limit
    theta_delta = update_theta(theta_delta)

    #time.sleep(0.01)
    
    return theta_delta



#Code for one iteration of IK, implemented on rover
def IK_rover():
    global theta
    global pos_delta
    
    #Read angle from encoder
    theta = input_theta()
    print(theta)

    #Read velocities from joysticks
    pos_delta = input_posdelta()

    #Calculate Frames for given theta (convert to rad for trig functions)
    frames = model.find_frames(np.deg2rad(theta))[0:7]
    
    #Return joint limit weights for given theta value
    W = weights.find_weights(theta)

    #Solve IK utilising the pseudo inverse jacobian method
    #Return theta_delta values after applying weights to joints
    theta_delta = jacobian.pseudo_inverse(frames, pos_delta, W)
    
    #Update the angles of each joint, uses weighting to reduce delta theta values to limit
    theta_delta = update_theta(theta_delta)

    #Output theta_delta
    output_thetadelta(theta_delta)

    print(theta)

    #Repeat every 10ms (100Hz)
    time.sleep(0.01)

    return



#Loop in simulation or implementation
def main():
    while sim==1:
        simulation()

    while sim==0:
        IK_rover()


if __name__ == "__main__":
    main()