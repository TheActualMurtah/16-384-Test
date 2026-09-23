import numpy as np
from common.robot_info import robot_info

def forward_kinematics_RR(theta):
    """
    forward_kinematics_RR

    Returns the forward kinematics for an RR robot given
    the joint angle positions [rad].

    The function returns N 3x3 frames, where N is the
    number of links.

    Each homogeneous transform describes the relationship from
    the base frame to start of the corresponding link.

    Hints
    - 'theta' is a vector. Individual angles can be selected
       using indices, e.g., theta1 = theta[0]
    """

    # Get information about the robot:
    robot = robot_info()
    # Extract length of the links
    l1, l2 = robot['link_lengths']

    # Ensure theta is a 1D array
    theta = np.squeeze(theta)

    # --------------- BEGIN STUDENT SECTION ----------------------------------
    # Define the transforms for all the frames below (replace each 'np.eye(3)'
    # with the corresponding matrix).  You can define helper variables, and
    # feel free to express your answers as products of matrices.
    theta_1, theta_2 = theta[0], theta[1]
    H_1_0 = np.array([[np.cos(theta_1), -np.sin(theta_1), 0], [np.sin(theta_1), np.cos(theta_1), 0], [0,0,1]])
    intermediate = H_1_0 @ np.array([[1,0,l1],[0,1,0],[0,0,1]])
    H_2_0 = intermediate @ np.array([[np.cos(theta_2), -np.sin(theta_2), 0],[np.sin(theta_2), np.cos(theta_2), 0],[0,0,1]])
    H_3_0 = H_2_0 @ np.array([[1,0,l2],[0,1,0],[0,0,1]])
    # --------------- END STUDENT SECTION ------------------------------------

    # Pack into a more readable format. DO NOT CHANGE!
    frames = {
        'H_1_0': H_1_0,
        'H_2_0': H_2_0,
        'H_3_0': H_3_0
    }
    return frames
