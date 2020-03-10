import numpy as np
from math import *
from scipy.linalg import fractional_matrix_power
import weights

#Finds the jacobian with the given frames and positions relative to the world frame
def find_jacobian(frames, joints):
    #Set up an empty 6 row matrix
    jacobian = np.zeros([joints,joints])
    z = np.zeros([3,6])

    #End effector position
    P_end=np.array(frames[-1][[0, 1, 2], [3, 3, 3]])

    #For each joint, solve a jacobian column vector using formula jvi = zi-1 x (Op - Oi-1)
    #Loop through each joint
    for i in range(joints):
        #Extract z column from each frame
        z[:,i] = np.array(frames[i][[0, 1, 2], [2, 2, 2]])

        #Find frame position respect to origin
        o_i = np.array(frames[i][[0, 1, 2], [3, 3, 3]])

        #Calculate Jacobian for Revolute joints (linear velocity)
        jacobian[0:3,i] = np.cross(z[:,i],(P_end - o_i))

        #Calculate Jacobian for angular velocity
        if joints == 6:
            jacobian[3:6,i] = z[0:3,i]

    return jacobian


def pseudo_inverse(frames, pos_delta, W, joints):
    #Utilises a Moore-Penrose pseudo inverse as an approximation. Can solve singular matrix jacobians where traditional inverse cannot.
    #Is faster and more efficient than an actual inverse, but not as accurate.
   
    jacobian = find_jacobian(frames, joints)
    
    #Weighted Jacobian
    jacobian_W = np.matmul(jacobian,fractional_matrix_power(W,0.5))

    #Finds pseudo inverse of the weighted jacobian utilising Moore-Penrose pseudo inverse
    pseudo_inverse = np.linalg.pinv(jacobian_W)

    #Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)

    #Convert raw angular velocity to degrees
    theta_delta = np.rad2deg(theta_delta)

    return theta_delta