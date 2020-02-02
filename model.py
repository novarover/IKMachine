import numpy as np
import main
from math import *


def revolute_joint(theta, d, a, alpha):
    # Returns the transformation frame for a standard revolute joint (theta/alpha must be radians)
    frame_array = np.mat([[cos(theta), -sin(theta), 0, a],
                          [sin(theta)*cos(alpha), cos(theta)*cos(alpha),
                           -sin(alpha), -sin(alpha)*d],
                          [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                          [0, 0, 0, 1]])
    return frame_array


def find_frames(theta):
    #This function returns the frames of each joint relative to the world frame for the current angles of each joint.

    # Here is where the model is defined.
    # First relative frames
    frame_01 = revolute_joint(theta[0], 0, 0, 0)  # base rotation
    frame_12 = revolute_joint(theta[1], 0, 0, pi/2.0)  # arm joint 1
    frame_23 = revolute_joint(theta[2], 0, 65, 0)  # arm joint 2
    # end effector left right
    frame_34 = revolute_joint(theta[3], 9, 42, 0)
    frame_45 = revolute_joint(theta[4], 11, 0, -pi/2.0)  # end effector up down
    frame_56 = revolute_joint(theta[5], 0, 0, -pi/2.0)  # continuous rotation
    #End effector from wrist
    frame_67 = revolute_joint(0,25,0,0)
    # Absolute frames are found through matrix multiplication
    frame_02 = np.matmul(frame_01, frame_12)
    frame_03 = np.matmul(frame_02, frame_23)
    frame_04 = np.matmul(frame_03, frame_34)
    frame_05 = np.matmul(frame_04, frame_45)
    frame_06 = np.matmul(frame_05, frame_56)
    frame_07 = np.matmul(frame_06, frame_67)

    # Rotation frame found from just the last 3 joints
    #frame_r_1 = np.matmul(frame_23, frame_34)
    #frame_r_2 = np.matmul(frame_r_1, frame_45)
    #frame_r = np.matmul(frame_r_2, frame_56)

    frames = [frame_01, frame_02, frame_03,
              frame_04, frame_05, frame_06, frame_07]

    return frames


# Conversion table is from encoder value to theta
conversion_table = [1.0,  # continuous rotation
                    1.0,  # arm joint 1
                    1.0,  # arm joint 2
                    1.0,  # end effector left right
                    1.0,  # end efffector up down
                    1.0,  # continuous rotation
                    ]


def find_theta(feedback_encoder):
    global conversion_table
    return np.multiply(feedback_encoder, conversion_table)


def find_encoder(output_theta):
    global conversion_table
    return np.true_divide(output_theta, conversion_table)
