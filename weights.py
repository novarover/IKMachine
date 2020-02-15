import numpy as np
import main
from math import *

def find_weights(theta):
    global qmin
    global qmax
    global H_prev

    qmax = main.qmax
    qmin = main.qmin
    H_prev = main.H_prev

    n = len(theta)
    m=2
    W = np.zeros((n, n))
    dH = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    for i in range(n):
        theta_i = theta[i]
        
        dH[i] = pow((qmax[i]-qmin[i]), 2)*(2*theta_i-qmax[i]-qmin[i]) / \
        (2*m*pow((qmax[i]-theta_i), 2)*pow((theta_i-qmin[i]), 2))
        #dH[i] = dH[i]/

        if (abs(dH[i])-abs(H_prev[i])>=0):
            W[i, i] = 1+abs(dH[i])
        else:
            W[i,i]=1

        H_prev[i]=dH[i]

    return W
