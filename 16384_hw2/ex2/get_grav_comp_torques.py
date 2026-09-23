import sys
sys.path.append("..")

import numpy as np
from common.robot_info import robot_info
from ex1.jacobian_link_ends_RR import jacobian_link_ends_RR
from ex1.jacobian_coms_RR import jacobian_coms_RR

def get_grav_comp_torques(theta, gravity):
    """
    Calculates the joint torques required to cancel out effects due to
    gravity.

    Args:
    theta (np.array): Joint angles
    gravity (np.array): Gravity vector (2x1)

    Returns:
    np.array: Joint torques for gravity compensation
    """
    # Get information about the robot:
    robot = robot_info()

    # Extract mass of the links, joint, and end effector [kg]
    m_link_1 = robot['link_masses'][0]
    m_link_2 = robot['link_masses'][1]
    m_joint_1 = robot['joint_masses'][0]
    m_joint_2 = robot['joint_masses'][1]
    m_end_effector = robot['end_effector_mass']
    theta = np.squeeze(theta)
    gravity = np.squeeze(gravity)

    # --------------- BEGIN STUDENT SECTION ----------------------------------
    # Use the Jacobian to calculate the joint torques to compensate for the
    # weights of the joints, links, and end effector (assuming the acceleration
    # due to gravity is given by 'gravity', and it is a 2x1 (column) vector).
    # Hint: You may find the jacobian_link_ends_RR and jacobian_coms_RR functions useful.
    # Your code here
    comJacob, endJacob = jacobian_coms_RR(theta), jacobian_link_ends_RR(theta)
    tau1 = comJacob["J_COM_1"].T @ -(m_link_1 * gravity)
    tau2 = comJacob["J_COM_2"].T @ -(m_link_2 * gravity)
    tau3 = endJacob["J_END_1"].T @ -(m_joint_2 * gravity)
    tau4 = endJacob["J_END_2"].T @ -(m_end_effector * gravity)
    tau = tau1 + tau2 + tau3 + tau4
    torque1 = tau[0]
    torque2 = tau[1]
    # --------------- END STUDENT SECTION ------------------------------------

    # Pack into a more readable format.
    return np.array([torque1, torque2])
