import numpy as np
import matplotlib.pyplot as plt

# Import the Robot class and required functions
from robot import Robot
from linear_workspace_trajectory import linear_workspace_trajectory
from visualize_trajectory import visualize_trajectory

# Verification Test
if __name__ == "__main__":
    # Create an RRR robot with unit link lengths
    link_lengths = np.array([1.0, 1.0, 1.0])
    link_masses = np.array([1.0, 1.0, 1.0])
    joint_masses = np.array([0.5, 0.5, 0.5])
    end_effector_mass = 0.5

    robot = Robot(link_lengths, link_masses, joint_masses, end_effector_mass)

    # Define start joint configuration and goal workspace position
    start_theta = np.array([np.pi / 4, np.pi / 2, np.pi / 2])
    goal_pos = np.array([1.0, 2.0])  # x and y coordinates

    # Number of steps in the trajectory
    num_points = 100

    # Generate the trajectory
    trajectory = linear_workspace_trajectory(robot, start_theta, goal_pos, num_points)

    # Visualize the trajectory
    visualize_trajectory(robot, trajectory)
