import numpy as np
import main
from math import *


def revolute_joint(theta, d, a, alpha):
    # Returns the transformation frame for a standard revolute joint
    frame_array = np.mat([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                          [sin(theta), cos(theta)*cos(alpha),
                           -cos(theta)*sin(alpha), a*sin(theta)],
                          [0, sin(alpha), cos(alpha), d],
                          [0, 0, 0, 1]])
    return frame_array


def find_frames(theta):
    # This function returns the frames of each joint relative to the world frame for the current angles of each joint.

    # Here is where the model is defined.
    # First relative frames
    frame_01 = revolute_joint(theta[0], 2, 0, pi/2.0)  # base rotation
    frame_12 = revolute_joint(theta[1], 0, 4, 0)  # arm joint 1
    frame_23 = revolute_joint(theta[2], 0, 3, 0)  # arm joint 2
    frame_34 = revolute_joint(theta[3], 0, 0, pi/2.0)  # end effector up down
    frame_45 = revolute_joint(theta[4], 0, 2, 0)  # end effector left right
    frame_56 = revolute_joint(0, 0, 0, theta[5])  # continuous rotation
    # Absolute frames are found through matrix multiplication
    frame_02 = np.matmul(frame_01, frame_12)
    frame_03 = np.matmul(frame_02, frame_23)
    frame_04 = np.matmul(frame_03, frame_34)
    frame_05 = np.matmul(frame_04, frame_45)
    frame_06 = np.matmul(frame_05, frame_56)

    frames = [frame_01, frame_02, frame_03,
              frame_04, frame_05, frame_06]

    return frames
