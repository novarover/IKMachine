import numpy as np
import main
from math import *

H_prev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#Function to calculate joint weights based on current theta values
def find_weights(theta):
    #Read joint limits and previous potential value
    qmax = main.qmax
    qmin = main.qmin
    

    #Initial Variables
    n = len(theta)
    m = 2
    W = np.zeros((n, n))
    dH = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    for i in range(n):
        #Calculate dH value for each angle
        theta_i = theta[i]
        
        dH[i] = pow((qmax[i]-qmin[i]), 2)*(2*theta_i-qmax[i]-qmin[i]) / \
        (2*m*pow((qmax[i]-theta_i), 2)*pow((theta_i-qmin[i]), 2))
        
        #Add direction information around limit
        if (abs(dH[i])-abs(H_prev[i])>=0):
            W[i, i] = 1+abs(dH[i])
        else:
            W[i,i]=1

        #Update H values for next iteration
        H_prev[i]=dH[i]

    return W
