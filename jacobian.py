import numpy as np
from math import *


def find_jacobian(frames, X, Y, Z):
    # Finds the jacobian with the given frames and positions relative to the world frame
    matrix_size = len(frames)

    # Set up an empty 3 row matrix
    jacobian = np.zeros([3, 0])

    # For each joint, solve a jacobian column vector using formula jvi = zi-1 x (Op - Oi-1)
    for i in range(matrix_size):
        # First joint utilizes z0 = [0,0,1] and origin position
        if (i == 0):
            jacobian = np.append(jacobian, (np.cross(
                np.array([[0, 0, 1]]), np.array([X[-1], Y[-1], Z[-1]]))).transpose(), axis=1)
        else:
            jacobian = np.append(jacobian, (np.cross(np.array(
                frames[i-1][[0, 1, 2], [2, 2, 2]]), np.array([X[-1]-X[i], Y[-1]-Y[i], Z[-1]-Z[i]]))).transpose(), axis=1)
    return jacobian


def pseudo_inverse(frames, X, Y, Z, pos_delta):

    jacobian = find_jacobian(frames, X, Y, Z)

    # Solving pseudo manually
    # pseudo_product = np.matmul(jacobian.transpose(), jacobian)
    # inverse_product = np.linalg.inv(pseudo_product)
    # pseudo_jacobian = np.matmul(inverse_product, jacobian)

    # Finds pseudo inverse of the jacobian utilising Moore-Penrose pseudo inverse
    pseudo_inverse = np.linalg.pinv(jacobian)

    # Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)
    return theta_delta
