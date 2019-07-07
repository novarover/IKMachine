import numpy as np
from math import *


def find_jacobian(frames, X, Y, Z):
    matrix_size = len(frames)
    jacobian = np.zeros([3, 0])
    for i in range(matrix_size):
        if (i == 0):
            jacobian = np.append(jacobian, (np.cross(
                np.array([[0, 0, 1]]), np.array([X[-1], Y[-1], Z[-1]]))).transpose(), axis=1)
        else:
            jacobian = np.append(jacobian, (np.cross(np.array(
                frames[i-1][[0, 1, 2], [2, 2, 2]]), np.array([X[-1]-X[i], Y[-1]-Y[i], Z[-1]-Z[i]]))).transpose(), axis=1)
    return jacobian


def pseudo_inverse(frames, X, Y, Z, pos_delta):
    jacobian = find_jacobian(frames, X, Y, Z)
    print(jacobian)
    # pseudo_product = np.matmul(jacobian.transpose(), jacobian)
    # print(pseudo_product)
    # inverse_product = np.linalg.inv(pseudo_product)
    # pseudo_jacobian = np.matmul(inverse_product, jacobian)
    pseudo_inverse = np.linalg.pinv(jacobian)
    theta_delta = np.matmul(pseudo_inverse, pos_delta)
    # print(pos_delta)
    return theta_delta
    # np.array(frames[i][[2, 2, 2], [0, 1, 2]])
