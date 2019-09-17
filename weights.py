import numpy as np
import model
from math import *

previous_weights = [0, 0, 0, 0, 0, 0, 0, 0, 0]


def find_weights(theta, jacobian):
    dimensions = len(theta)
    weight = np.zeros((dimensions, dimensions))
    for i in range(dimensions):
        theta_i = theta[i]
        theta_max = model.max_limit[i]
        theta_min = model.min_limit[i]
        H = pow((theta_max-theta_min), 2)*(2*theta_i-theta_max-theta_min) / \
            (2*dimensions*pow((theta_max-theta_i), 2)*pow((theta_i-theta_min), 2))
        weight[i, i] = 1+abs(H)
    return weight
