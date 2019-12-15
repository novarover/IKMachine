import numpy as np
from math import *
import weights


def find_jacobian(frames, X, Y, Z, joints):
    # Finds the jacobian with the given frames and positions relative to the world frame

    # Set up an empty 6 row matrix
    jacobian = np.zeros([joints, 0])

    # For each joint, solve a jacobian column vector using formula jvi = zi-1 x (Op - Oi-1)
    for i in range(joints):
        # First joint utilizes z0 = [0,0,1] and origin position
        if (i == 0):
            z = np.array([[0, 0, 1]])
            jacobian_column = np.cross(z, np.array(
                [X[-1], Y[-1], Z[-1]])).transpose()
            if joints > 3:
                jacobian_column = np.append(
                    jacobian_column, z.transpose(), axis=0)
            jacobian = np.append(jacobian, jacobian_column, axis=1)
        else:
            z = np.array(frames[i-1][[0, 1, 2], [2, 2, 2]])
            jacobian_column = np.cross(np.array(
                frames[i-1][[0, 1, 2], [2, 2, 2]]), np.array([X[-1]-X[i], Y[-1]-Y[i], Z[-1]-Z[i]])).transpose()
            if joints > 3:
                jacobian_column = np.append(
                    jacobian_column, z.transpose(), axis=0)
            jacobian = np.append(jacobian, jacobian_column, axis=1)
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
    #print(jacobian)
    # Solving pseudo manually
    # pseudo_product = np.matmul(jacobian.transpose(), jacobian)
    # inverse_product = np.linalg.inv(pseudo_product)
    # pseudo_jacobian = np.matmul(inverse_product, jacobian)

    # Finds pseudo inverse of the jacobian utilising Moore-Penrose pseudo inverse
    pseudo_inverse = np.linalg.pinv(jacobian)

    # Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)
    return theta_delta


def pseudo_inverse_weights(frames, X, Y, Z, pos_delta, joints, theta):
    jacobian = find_jacobian(frames, X, Y, Z, joints)
    # print('ttt')
    # print(jacobian)
    weight = weights.find_weights(theta, jacobian)
    # sqrt_weight = np.sqrt(weight)
    # inverse_sqrt = np.linalg.inv(sqrt_weight)
    # jw = np.matmul(jacobian, inverse_sqrt)

    inverse_weight = np.linalg.inv(weight)
    jacobian_t = jacobian.transpose()
    print(jacobian)
    print(jacobian_t)
    part_1 = np.matmul(inverse_weight, jacobian_t)
    weighted_jacobian_product = np.matmul(
        jacobian, part_1)
    inverse_weight = np.linalg.inv(weighted_jacobian_product)
    pseudo_inverse = np.matmul(
        np.matmul(inverse_weight, jacobian_t), inverse_weight)

    # pseudo = np.linalg.pinv(jw)
    # pseudo_inverse = np.matmul(inverse_sqrt, pseudo)

    # Finds pseudo inverse of the jacobian utilising Moore-Penrose pseudo inverse
    # pseudo_inverse=np.linalg.pinv(jacobian)
    # Calculate required change in angles for each variable joint
    theta_delta = np.matmul(pseudo_inverse, pos_delta)

    return theta_delta


def inverse(frames, X, Y, Z, pos_delta):
    # Look at utilising numpy's built in algebra solver?
    pass


def linear_solve(frames, X, Y, Z, pos_delta, joints):
     # Calls a gesv lapack routine and solves using forward and back substitution
    jacobian = find_jacobian(frames, X, Y, Z, joints)
    # Gives a pseudo version of the jacobian to solve for singular matrices
    pseudo_jacobian = np.matmul(jacobian.transpose(), jacobian)

    # Utilises the in-built numpy line
    # ar algebra solver, takes in A and b from A*x=b to return x
    theta_delta = pseudo_inverse(frames, X, Y, Z, pos_delta, joints)
    # try:
    #    theta_delta = np.linalg.solve(jacobian, pos_delta)
    # except:
    #    theta_delta = pseudo_inverse(frames, X, Y, Z, pos_delta, joints)
    #    print("Except")
    return theta_delta
