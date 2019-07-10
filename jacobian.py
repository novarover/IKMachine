import numpy as np
from math import *


def find_jacobian(frames, X, Y, Z, joints):
    # Finds the jacobian with the given frames and positions relative to the world frame

    # Set up an empty 3 row matrix
    jacobian = np.zeros([joints, 0])

    # For each joint, solve a jacobian column vector using formula jvi = zi-1 x (Op - Oi-1)
    for i in range(joints):
        # First joint utilizes z0 = [0,0,1] and origin position
        if (i == 0):
            z = np.array([[0, 0, 1]])
            jacobian_column = np.cross(z, np.array(
                [X[-1], Y[-1], Z[-1]])).transpose()
            jacobian_column = np.append(jacobian_column, z.transpose(), axis=0)
            jacobian = np.append(jacobian, jacobian_column, axis=1)
        else:
            z = np.array(frames[i-1][[0, 1, 2], [2, 2, 2]])
            jacobian_column = np.cross(np.array(
                frames[i-1][[0, 1, 2], [2, 2, 2]]), np.array([X[-1]-X[i], Y[-1]-Y[i], Z[-1]-Z[i]])).transpose()
            jacobian_column = np.append(jacobian_column, z.transpose(), axis=0)
            jacobian = np.append(jacobian, jacobian_column, axis=1)
            # print(jacobian_column)
    return jacobian


def transpose(frames, X, Y, Z, pos_delta, joints):
    # Simplest and quickest approach, utilises the transpose as an approximation for the inverse jacobian.
    jacobian = find_jacobian(frames, X, Y, Z, joints)
    transpose = jacobian.transpose()
    theta_delta = np.matmul(transpose, pos_delta)
    return theta_delta


def pseudo_inverse(frames, X, Y, Z, pos_delta, joints):
    # Utilises a Moore-Penrose pseudo inverse as an approximation. Can solve singular matrix jacobians where traditional inverse cannot.
    # Is faster and more efficient than an actual inverse, but not as accurate.
    jacobian = find_jacobian(frames, X, Y, Z, joints)

    # Solving pseudo manually
    # pseudo_product = np.matmul(jacobian.transpose(), jacobian)
    # inverse_product = np.linalg.inv(pseudo_product)
    # pseudo_jacobian = np.matmul(inverse_product, jacobian)

    # Finds pseudo inverse of the jacobian utilising Moore-Penrose pseudo inverse
    pseudo_inverse = np.linalg.pinv(jacobian)

    # Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)
    return theta_delta


def inverse(frames, X, Y, Z, pos_delta):
    # Look at utilising numpy's built in algebra solver?
    pass
