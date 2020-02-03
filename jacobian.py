import numpy as np
from math import *
import weights


def find_jacobian(frames, X, Y, Z, joints):
    # Finds the jacobian with the given frames and positions relative to the world frame

    # Set up an empty 6 row matrix
    jacobian = np.zeros([6,6])
    z = np.zeros([3,6])

    # For each joint, solve a jacobian column vector using formula jvi = zi-1 x (Op - Oi-1)

    #End effector position
    P_end=np.array([[X[-1], Y[-1], Z[-1]]])

    #Loop through each theta
    for i in range(joints):
        #Extract z column from each frame
        z[:,i] = np.array(frames[i][[0, 1, 2], [2, 2, 2]])
        #Find frame position respect to origin
        o_i = np.array(frames[i][[0, 1, 2], [3, 3, 3]])
        #Calculate Jacobian for Revolute joints (linear velocity)
        #print(jacobian[0:3,i])
        #print(z[:,i])
        #print(P_end - o_i)
        jacobian[0:3,i] = np.cross(z[:,i],(P_end - o_i))
        #Calculate Jacobian for angular velocity
        jacobian[3:6,i] = z[0:3,i]

    
    #print(z)
    
    #print(P_end)
    
    #print(jacobian)

    # for i in range(joints):
    #     # First joint utilizes z0 = [0,0,1] and origin position
    #     if (i == 0):
    #         z = np.array([[0, 0, 1]])
    #         jacobian_column = np.cross(z, np.array(
    #             [X[-1], Y[-1], Z[-1]])).transpose()
    #         if joints > 3:
    #             jacobian_column = np.append(
    #                 jacobian_column, z.transpose(), axis=0)
    #         jacobian = np.append(jacobian, jacobian_column, axis=1)
    #     else:
    #         z = np.array(frames[i-1][[0, 1, 2], [2, 2, 2]])
    #         jacobian_column = np.cross(np.array(
    #             frames[i-1][[0, 1, 2], [2, 2, 2]]), np.array([X[-1]-X[i], Y[-1]-Y[i], Z[-1]-Z[i]])).transpose()
    #         if joints > 3:
    #             jacobian_column = np.append(
    #                 jacobian_column, z.transpose(), axis=0)
    #         jacobian = np.append(jacobian, jacobian_column, axis=1)

    return jacobian


def pseudo_inverse(frames, X, Y, Z, pos_delta, joints):
    # Utilises a Moore-Penrose pseudo inverse as an approximation. Can solve singular matrix jacobians where traditional inverse cannot.
    # Is faster and more efficient than an actual inverse, but not as accurate.
    jacobian = find_jacobian(frames, X, Y, Z, joints)
    #print(jacobian)
   
    # Finds pseudo inverse of the jacobian utilising Moore-Penrose pseudo inverse
    pseudo_inverse = np.linalg.pinv(jacobian)

    # Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)

    #Convert raw angular velocity to degrees
    theta_delta = np.rad2deg(theta_delta)
    return theta_delta