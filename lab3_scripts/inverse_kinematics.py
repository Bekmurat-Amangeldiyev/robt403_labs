#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy

# Define link lengths
L1, L2, L3 = 13.7, 13.7, 4.5

# Function to compute forward kinematics for a 3-DOF planar robot
def forward_kinematics(theta1, theta2, theta3):
    x1, y1 = 0, 0  # Base position
    x2 = L1 * np.cos(theta1)
    y2 = L1 * np.sin(theta1)
    x3 = x2 + L2 * np.cos(theta1 + theta2)
    y3 = y2 + L2 * np.sin(theta1 + theta2)
    x_end = x3 + L3 * np.cos(theta1 + theta2 + theta3)
    y_end = y3 + L3 * np.sin(theta1 + theta2 + theta3)
    
    return (x1, y1), (x2, y2), (x3, y3), (x_end, y_end)

# Inverse Kinematics to find thetas based on desired end effector position (x_end, y_end)
def inverse_kinematics(x_end, y_end):
    # Check if the point is reachable (inside the workspace)
    distance = np.sqrt(x_end**2 + y_end**2)
    if distance > (L1 + L2 + L3):
        print("Point is outside the workspace.")
        return None, None, None

    # Calculate the wrist position (wrist position is the end-effector without the final link L3)
    wrist_x = x_end - L3 * np.cos(np.arctan2(y_end, x_end))
    wrist_y = y_end - L3 * np.sin(np.arctan2(y_end, x_end))

    # Calculate theta2 using the cosine law
    D = (wrist_x**2 + wrist_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D < -1 or D > 1:
        print("Point is outside the workspace.")
        return None, None, None

    theta2 = np.arccos(D)

    # Calculate theta1
    theta1 = np.arctan2(wrist_y, wrist_x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    # Calculate theta3 based on orientation
    theta3 = np.arctan2(y_end - wrist_y, x_end - wrist_x) - (theta1 + theta2)

    return theta1, theta2, theta3

# Function to plot manipulator at each step
def plot_manipulator(theta1, theta2, theta3, target_x, target_y):
    (x1, y1), (x2, y2), (x3, y3), (calculated_x_end, calculated_y_end) = forward_kinematics(theta1, theta2, theta3)

    # Clear the plot and update it with the new positions
    plt.clf()
    plt.plot([x1, x2], [y1, y2], 'b-', linewidth=2, label='Link 1')
    plt.plot([x2, x3], [y2, y3], 'g-', linewidth=2, label='Link 2')
    plt.plot([x3, calculated_x_end], [y3, calculated_y_end], 'r-', linewidth=2, label='Link 3')
    
    # Plot the target position
    plt.scatter(target_x, target_y, color='magenta', label='Target Position')

    plt.title('Manipulator Moving to Desired Position')
    plt.xlabel('X position (cm)')
    plt.ylabel('Y position (cm)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.pause(0.05)  # Pause for animation

# Function to interpolate between initial and final joint angles
def interpolate_motion(theta_initial, theta_final, steps):
    theta_diff = np.array(theta_final) - np.array(theta_initial)
    for i in range(steps):
        theta_current = theta_initial + theta_diff * (i / steps)
        yield theta_current

def simulate_motion():
    rospy.init_node('manipulator_simulation', anonymous=True)
    rospy.loginfo("Simulating manipulator motion...")

    # Desired end effector position
    target_x = float(input("Enter desired X position (cm): "))
    target_y = float(input("Enter desired Y position (cm): "))

    # Current manipulator position (initial angles)
    theta_initial = [0.0, 0.0, 0.0]  # Initial configuration: straight arm
    theta1, theta2, theta3 = inverse_kinematics(target_x, target_y)

    if theta1 is not None and theta2 is not None and theta3 is not None:
        theta_final = [theta1, theta2, theta3]

        plt.figure(figsize=(8, 8))

        # Interpolate the motion in 100 steps
        for theta_current in interpolate_motion(theta_initial, theta_final, steps=100):
            plot_manipulator(theta_current[0], theta_current[1], theta_current[2], target_x, target_y)

        plt.show()

    else:
        print("The point is outside the workspace or cannot be reached.")

if __name__ == '__main__':
    try:
        simulate_motion()
    except rospy.ROSInterruptException:
        pass

