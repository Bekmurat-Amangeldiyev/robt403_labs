#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt

def cubic_coefficients(theta_0, theta_f, t_0, t_f, vel_0=0, vel_f=0):
    A = np.array([[1, t_0, t_0 ** 2, t_0 ** 3],
                  [0, 1, 2 * t_0, 3 * t_0 ** 2],
                  [1, t_f, t_f ** 2, t_f ** 3],
                  [0, 1, 2 * t_f, 3 * t_f ** 2]])
    B = np.array([theta_0, vel_0, theta_f, vel_f])
    coefficients = np.linalg.solve(A, B)
    return coefficients

def cubic_trajectory(coefficients, t):
    a0, a1, a2, a3 = coefficients
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

def inverse_kinematics_with_orientation(X, Y, theta, L1, L2, L3):
    x = X - L3 * np.cos(theta)
    y = Y - L3 * np.sin(theta)

    cos_q2 = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1, 1)
    q2 = np.arccos(cos_q2)

    K1 = L1 + L2 * np.cos(q2)
    K2 = L2 * np.sin(q2)
    q1 = np.arctan2(y, x) - np.arctan2(K2, K1)

    q1 = np.arctan2(np.sin(q1), np.cos(q1))
    q3 = theta - q1 - q2
    q3 = np.arctan2(np.sin(q3), np.cos(q3))

    return q1, q2, q3

def forward_kinematics(q1, q2, q3, L1, L2, L3):
    x1 = L1 * np.cos(q1)
    y1 = L1 * np.sin(q1)

    x2 = x1 + L2 * np.cos(q1 + q2)
    y2 = y1 + L2 * np.sin(q1 + q2)

    x3 = x2 + L3 * np.cos(q1 + q2 + q3)
    y3 = y2 + L3 * np.sin(q1 + q2 + q3)

    return (0, 0), (x1, y1), (x2, y2), (x3, y3)

def plot_manipulator(q1, q2, q3, L1, L2, L3):
    joint1, joint2, joint3, end_effector = forward_kinematics(q1, q2, q3, L1, L2, L3)

    plt.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], 'b-', label='Link 1')
    plt.plot([joint2[0], joint3[0]], [joint2[1], joint3[1]], 'g-', label='Link 2')
    plt.plot([joint3[0], end_effector[0]], [joint3[1], end_effector[1]], 'r-', label='Link 3')

    plt.plot(joint1[0], joint1[1], 'bo', label='Joint 1')
    plt.plot(joint2[0], joint2[1], 'go', label='Joint 2')
    plt.plot(joint3[0], joint3[1], 'ro', label='Joint 3')
    plt.plot(end_effector[0], end_effector[1], 'mo', label='End-Effector')

    plt.arrow(end_effector[0], end_effector[1], 0.2 * np.cos(q1 + q2 + q3), 0.2 * np.sin(q1 + q2 + q3),
              head_width=0.1, head_length=0.1, fc='m', ec='m')

    plt.title("Manipulator Configuration")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.pause(0.01)  # Ensure plot is updated

def send_joint_angles(X, Y, theta, L1, L2, L3):
    # Initialize ROS node
    rospy.init_node('joint_angle_sender', anonymous=True)

    # Publishers for each joint controller
    joint1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)

    # Perform inverse kinematics to find joint angles
    q1, q2, q3 = inverse_kinematics_with_orientation(X, Y, theta, L1, L2, L3)

    # Publish the joint angles
    joint1_pub.publish(q1)
    joint2_pub.publish(q2)
    joint3_pub.publish(q3)

    # Log the joint angles
    rospy.loginfo("Published joint angles: q1={}, q2={}, q3={}".format(q1, q2, q3))


    # Plot the manipulator configuration
    plot_manipulator(q1, q2, q3, L1, L2, L3)
    plt.show(block=True)  # Keep the plot open until closed manually

if __name__ == '__main__':
    try:
        # Link lengths for your 3RRR manipulator
        L1 = 0.675 * 2
        L2 = 0.675 * 2
        L3 = 3.167 - 0.675 * 4  # Adjust as per your robot's design

        # Example X, Y, and theta values
        X = 10.0  # X position
        Y = 15.0  # Y position
        theta = np.pi / 4  # Orientation angle

        # Call the function to send joint angles and plot the manipulator
        send_joint_angles(X, Y, theta, L1, L2, L3)

    except rospy.ROSInterruptException:
        pass

