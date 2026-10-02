import numpy as np
import matplotlib.pyplot as plt

def visualize_trajectory(robot, trajectory):
    """
    Visualizes the end effector path and joint angles during a trajectory.

    Parameters:
    robot : Robot object with an end_effector method
    trajectory : numpy array of shape (num_joints, num_time_steps)
    """
    # Calculate end effector path through entire log
    n = trajectory.shape[1]
    x = np.zeros(n)
    y = np.zeros(n)
    for i in range(n):
        ee = robot.end_effector(trajectory[:, i])
        x[i] = ee[0]
        y[i] = ee[1]
    
    # Plot path data
    plt.figure()
    plt.plot(x, y, 'k-', linewidth=1)
    plt.title('Plot of end effector position during trajectory')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.show()

    # Plot angle data
    plt.figure()
    plt.plot(range(1, trajectory.shape[1]+1), trajectory.T, linewidth=1)
    plt.title('Plot of joint positions during trajectory')
    plt.xlabel('t')
    plt.ylabel('θ values')
    plt.show()
