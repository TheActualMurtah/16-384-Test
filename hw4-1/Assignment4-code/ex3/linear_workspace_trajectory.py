import numpy as np

def inverse_kinematics(robot, initial_theta, desired_pos):
    """
    Placeholder inverse kinematics function.
    Replace this with the actual inverse kinematics implementation.
    """
    # For demonstration purposes, we'll assume this function exists.
    # In practice, students will need to implement this or it will be provided.
    return robot.inverse_kinematics(initial_theta, desired_pos)

def linear_workspace_trajectory(robot, start_theta, goal_pos, num_points):
    """
    Returns a matrix of joint angles, where each column represents a single
    timestamp. This matrix is of shape (num_joints, num_points).

    Using these joint angles, the end effector linearly interpolates from the
    location given by start_theta to goal_pos, in the workspace.

    Parameters:
    robot : Robot object for IK calculations
    start_theta : numpy array of shape (num_joints,)
    goal_pos : numpy array representing the goal position (and orientation) in workspace
    num_points : int

    Returns:
    trajectory : numpy array of shape (num_joints, num_points)
    """
    # --------------- BEGIN STUDENT SECTION ----------------------------------
    trajectory = np.zeros((len(start_theta), num_points))

    ee = robot.end_effector(start_theta)
    start_pos = ee[:len(goal_pos)]

    ws_trajectory = np.zeros((len(goal_pos), num_points))

    # TODO: Compute ws_trajectory by linearly interpolating between start_pos and goal_pos
    # HINT: You may use numpy.linspace
    for i in range(len(goal_pos)):
        ws_trajectory[i, :] = np.linspace(start_pos[i], goal_pos[i], num_points)

    # We know the first column:
    trajectory[:, 0] = start_theta

    # Compute the rest of the trajectory
    for col in range(1, num_points):
        # TODO: Fill in trajectory[:, col] here
        # HINT: Use inverse kinematics with the previous joint angles as initial guess
        target = ws_trajectory[:, col]
        idk = trajectory[:, col - 1]
        trajectory[:, col] = robot.inverse_kinematics(idk, target)

    # --------------- END STUDENT SECTION ------------------------------------

    return trajectory
