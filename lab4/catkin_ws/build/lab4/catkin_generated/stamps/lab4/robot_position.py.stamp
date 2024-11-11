import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# Lists to store x and y positions for plotting
x_positions = []
y_positions = []

# Callback function for receiving position data
def position_callback(data):
    x_positions.append(data.data[0])
    y_positions.append(data.data[1])

def live_plot():
    rospy.init_node('live_plot', anonymous=True)
    rospy.Subscriber("robot_position", Float64MultiArray, position_callback)

    # Initialize the plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(0, 0, color='red', marker='x', s=100, label="Center (O)")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("Live Robot End-Effector Path - Circle with Radius 0.3 (Clockwise)")
    ax.legend()
    ax.grid(True)
    ax.axis("equal")

    while not rospy.is_shutdown():
        # Check if there is data to plot
        if x_positions and y_positions:
            # Clear the previous path and plot the updated path
            ax.plot(x_positions, y_positions, marker='o', linestyle='-', color='blue', markersize=5)
            # Plot the start point only if it exists
            ax.scatter(x_positions[0], y_positions[0], color='green', marker='o', s=100, label="Start Point (12 o'clock)")
        
        plt.draw()
        plt.pause(0.05)  # Pause briefly to update the plot

    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep the plot open after ROS shuts down

if __name__ == '__main__':
    live_plot()

