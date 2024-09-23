#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>  // for sine function

// Function to generate a sine wave signal
double generateSineWave(double amplitude, double frequency, ros::Time start_time, ros::Time current_time) {
    double time_since_start = (current_time - start_time).toSec();
    return amplitude * sin(2 * M_PI * frequency * time_since_start);  // Sine wave formula
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sine_wave_controller");
    ros::NodeHandle nh;

    // Publishers for base joint and end-effector joint
    ros::Publisher base_joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    ros::Publisher end_effector_joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);

    ros::Rate loop_rate(10);  // 10 Hz loop rate

    ros::Time start_time = ros::Time::now();
    double amplitude = 1.0;  // Amplitude of the sine wave
    double frequency = 0.2;  // Frequency in Hz (adjust based on your needs)

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        
        // Generate sine wave signals for base and end-effector joints
        std_msgs::Float64 base_joint_msg;
        std_msgs::Float64 end_effector_joint_msg;

        base_joint_msg.data = generateSineWave(amplitude, frequency, start_time, current_time);
        end_effector_joint_msg.data = generateSineWave(amplitude, frequency, start_time, current_time);

        // Publish the commands
        base_joint_pub.publish(base_joint_msg);
        end_effector_joint_pub.publish(end_effector_joint_msg);

        // Log the state for debugging
        ROS_INFO("Base Joint Command: %.2f, End Effector Command: %.2f", base_joint_msg.data, end_effector_joint_msg.data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

