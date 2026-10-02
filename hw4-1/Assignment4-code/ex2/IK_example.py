from robot import Robot
import numpy as np

# Example 1
robot = Robot(
    link_lengths=np.array([1, 1]),
    link_masses=np.array([1, 1]),
    joint_masses=np.array([1, 1]),
    end_effector_mass=1
)
initial_angles = np.array([np.pi / 4, np.pi / 2])
goal = np.array([-np.sqrt(2), np.sqrt(2)])
final_angles = robot.inverse_kinematics(initial_angles, goal)
print("Final joint angles:", final_angles)
print("End effector position:", robot.end_effector(final_angles))

# Verify that the end effector position matches the goal

print("----------------------------------------------------------")

# Example 2
robot = Robot(
    link_lengths=np.array([1, 1, 1]),
    link_masses=np.array([1, 1, 1]),
    joint_masses=np.array([1, 1, 1]),
    end_effector_mass=1
)
initial_angles = np.array([np.pi / 2, 0., 0.])
goal = np.array([1.6, 1.6])
final_angles = robot.inverse_kinematics(initial_angles, goal)
print("Final joint angles:", final_angles)
print("End effector position:", robot.end_effector(final_angles))


print("---------------------------------------------------------")

# Example 3
robot = Robot(
    link_lengths=np.array([1, 1, 1]),
    link_masses=np.array([1, 1, 1]),
    joint_masses=np.array([1, 1, 1]),
    end_effector_mass=1
)
initial_angles = np.array([-np.pi / 2, 0., 0.])
goal = np.array([1.6, 1.6, np.pi/2])
final_angles = robot.inverse_kinematics(initial_angles, goal)
print("Final joint angles:", final_angles)
print("End effector position:", robot.end_effector(final_angles))
