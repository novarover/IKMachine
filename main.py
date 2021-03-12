#Import libraries
import numpy as np
import jacobian
import time
import model
import weights
from math import *


# Main class for Inverse Kinematics
class IKMachine:

    # Define whether to use the plotter or not
    USE_PLOTTER = True

    # Define the starting values of theta (degrees)
    theta_1 = 0 
    theta_2 = 90  
    theta_3 = -90
    theta_4 = 0
    theta_5 = -90
    theta_6 = 0

    # Define global variables
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
    adpt_frame = 0
    #Toggle for joint limits
    joint_limit = 0


    # Initialisation function
    def __init__ (self, USE_PLOTTER = False):
        self.USE_PLOTTER = USE_PLOTTER

        if (USE_PLOTTER):
            print("Initialised the IK class with plotting.")
        else:
            print("Initialised the Inverse Kinematics class for code")


    #Function to clamp joint if it exceeds joint limit by 2 degrees
    def clamp(self, theta):

        limit = 2  #Testing Variable
        dimensions = len(theta)
        H = np.zeros((dimensions, dimensions))
        
        for i in range(dimensions):
            if (theta[i]>=self.qmax[i]+limit)|(theta[i]<=self.qmin[i]-limit):
                H[i,i]=0
                print('Clamped')    
            else:  
                H[i,i]=1
        return H


    #Function to wrap any angle to[-180,180]
    def wrapTo180(self, angle):
        angle = angle % 360
        angle = (angle + 360) % 360

        if (angle > 180):
            angle = angle - 360

        return angle


    #Function to update theta values using scaled delta_theta 
    def update_theta(self, theta_delta_raw):

        #Limit change in theta values using proportional weights (alpha)
        max_delta = np.amax(abs(theta_delta_raw))
        alpha = 1
        new_theta = [0.0,0.0,0.0,0.0,0.0,0.0]
        theta_delta = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        
        #Check for joint limit breach and clamp if required
        if self.clamping == 1:
            H = clamp(theta)
            theta_delta_raw = np.dot(H,theta_delta_raw)

        #Compare max delta_theta value to max allowed value (deg/s)
        if max_delta > self.theta_max: 
            alpha = 1/(float(max_delta/theta_max))
        
        for i in range(len(theta_delta_raw)):
            theta_delta[i] = theta_delta_raw[i]*alpha #Scale by alpha
            new_theta[i] = self.theta[i]+theta_delta[i]    #Update angle values
            new_theta[i] = self.wrapTo180(new_theta[i])    #Wrap new angles to [-180,180]
        
        #Assign updated theta values (if sim)
        if self.USE_PLOTTER:
            self.theta = new_theta 

        return theta_delta


    # Function to set the theta
    def set_theta (self, new_theta):
        self.theta = new_theta




    #Looping function for model simulation
    def solve(self, velocity_input):
        
        # Get the velocity input
        if self.USE_PLOTTER:
            import plotter
            self.pos_delta = plotter.pos_delta
        else:
            self.pos_delta = velocity_input

        #Adaptive reference frame option
        if self.adpt_frame == 1:
            self.pos_adapt[0:3] = model.xyz_reframe(self.pos_delta[0:3], self.theta[0])  #Read intial base rotation angle to pass into reframe function
        else:
            self.pos_adapt[0:3] = self.pos_delta[0:3]


        #Calculate Frames for given theta (convert to rad for trig functions)
        frames = model.find_frames(np.deg2rad(self.theta))[0:7]
        
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
        if self.USE_PLOTTER:
            plotter.plot(X, Y, Z)

        #Return joint limit weights for given theta value
        if self.joint_limit == 1:
            W = weights.find_weights(self.theta)
        else:
            W = np.identity(6)

        #Solve IK utilising the pseudo inverse jacobian method
        #Return theta_delta values after applying weights to joints
        self.theta_delta = jacobian.pseudo_inverse(frames, self.pos_adapt, W)
        
        #Update the angles of each joint, uses weighting to reduce delta theta values to limit
        self.theta_delta = self.update_theta(self.theta_delta)

        #time.sleep(0.01)
        #print(W[0,0])
        return self.theta_delta


# Main function
if __name__ == "__main__":
    IK = IKMachine()

    # If using plotter
    if (IK.USE_PLOTTER):
        while True:
            IK.solve(None)