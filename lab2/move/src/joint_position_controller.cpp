#include "ros/ros.h"
#include "std_msgs/Float64.h"
// Almatikx code for listener
//GOODLUCK
// Global variable to store the previous value
double previous_value = -std::numeric_limits<double>::infinity();  // Start with a very low value

ros::Publisher joint_pub;  // Publisher to control the joint

// Callback function to process incoming values
void valueCallback(const std_msgs::Float64::ConstPtr& msg) {
    double current_value = msg->data;

    // Check if the current value is higher than the previous one
    if (current_value > previous_value) {
        // Publish the new value to the joint controller topic
        std_msgs::Float64 joint_command;
        joint_command.data = current_value;
        joint_pub.publish(joint_command);

        ROS_INFO("Published new joint command: %f", current_value);

        // Update the previous value
        previous_value = current_value;
    } else {
        ROS_INFO("Incoming value %f is not higher than the previous value %f", current_value, previous_value);
    }
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "joint_position_controller");
    ros::NodeHandle nh;

    // Create a publisher to control the joint
    joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 100);

    // Subscribe to the incoming value topic (you can change the topic name to your input topic)
    ros::Subscriber sub = nh.subscribe("/incoming_values", 1000, valueCallback);

    // Spin to keep the node running and listen for incoming data
    ros::spin();

    return 0;
}

