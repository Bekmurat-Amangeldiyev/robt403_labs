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

def calculate_workspace():
    rospy.init_node('workspace_calculation', anonymous=True)
    rospy.loginfo("Calculating workspace with joint limits...")

    # Joint limits in radians
    joint_min = -np.pi / 2  # -90 degrees
    joint_max = np.pi / 2   # 90 degrees

    # Generate joint angles within their limits
    theta1_range = np.linspace(joint_min, joint_max, 100)
    theta2_range = np.linspace(joint_min, joint_max, 100)
    theta3_range = np.linspace(joint_min, joint_max, 100)

    workspace_x = []
    workspace_y = []
    
    plt.figure(figsize=(8, 8))

    # Randomly select a configuration for the manipulator
    random_theta1 = np.random.choice(theta1_range)
    random_theta2 = np.random.choice(theta2_range)
    random_theta3 = np.random.choice(theta3_range)
    
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            for theta3 in theta3_range:
                # Compute positions
                (x1, y1), (x2, y2), (x3, y3), (x_end, y_end) = forward_kinematics(theta1, theta2, theta3)
                
                # Store end effector positions
                workspace_x.append(x_end)
                workspace_y.append(y_end)
                
                # Plot the randomly selected configuration of the manipulator's links
                if theta1 == random_theta1 and theta2 == random_theta2 and theta3 == random_theta3:
                    plt.plot([x1, x2], [y1, y2], 'b-', linewidth=2)  # Link 1
                    plt.plot([x2, x3], [y2, y3], 'g-', linewidth=2)  # Link 2
                    plt.plot([x3, x_end], [y3, y_end], 'r-', linewidth=2)  # Link 3

    # Fill the workspace area with blue
    plt.fill(workspace_x, workspace_y, color='blue', alpha=0.3)

    plt.title('Workspace of the 3-DOF Planar Manipulator with Joint Limits')
    plt.xlabel('X position (cm)')
    plt.ylabel('Y position (cm)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend(['Workspace Points', 'Manipulator Configuration'])
    plt.show()

if __name__ == '__main__':
    try:
        calculate_workspace()
    except rospy.ROSInterruptException:
        pass

