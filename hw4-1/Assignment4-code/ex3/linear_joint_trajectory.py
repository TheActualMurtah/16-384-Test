import numpy as np

def linear_joint_trajectory(start_theta, goal_theta, num_points):
    """
    Returns a matrix of joint angles, where each column represents a single
    timestamp. This matrix is of shape (num_joints, num_points).

    Each joint angle linearly interpolates from its start value to its end
    value, in joint space.

    Parameters:
    start_theta : numpy array of shape (num_joints,)
    goal_theta : numpy array of shape (num_joints,)
    num_points : int

    Returns:
    trajectory : numpy array of shape (num_joints, num_points)
    """
    # --------------- BEGIN STUDENT SECTION ----------------------------------

    trajectory = np.zeros((len(start_theta), num_points))

    # TODO: Fill in the code to compute the trajectory
    # HINT: You may find numpy.linspace helpful
    trajectory = np.linspace(start_theta, goal_theta, num=num_points, axis=1)

    # --------------- END STUDENT SECTION ------------------------------------

    return trajectory
