#include "ros/ros.h" 
#include "std_msgs/Float64.h" 
#include <signal.h> 
 
ros::Publisher pub_joint1; 
ros::Publisher pub_joint2; 
ros::Publisher pub_joint3; 
ros::Publisher pub_joint4; 
ros::Publisher pub_joint5; 
 
// Signal handler for catching CTRL + C 
void signalHandler(int signum) { 
    ROS_INFO("Shutting down... Moving all joints to initial 0 position"); 
 	
    // Create the message for setting joints to 0 
    std_msgs::Float64 reset_msg; 
    reset_msg.data = 0.0; 
 
    // Publish 0 to all joint topics 
    pub_joint1.publish(reset_msg); 
    pub_joint2.publish(reset_msg); 
    pub_joint3.publish(reset_msg); 
    pub_joint4.publish(reset_msg); 
    pub_joint5.publish(reset_msg); 
 
    // Give it some time to send the message 
    ros::Duration(1.0).sleep(); 
 
    // Shutdown the ROS node 
    ros::shutdown(); 
} 
 
int main(int argc, char **argv) { 
    ros::init(argc, argv, "snake_movement"); 
    ros::NodeHandle nh; 
 
    // Publishers for each joint 
    pub_joint1 = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 100); 
    pub_joint2 = nh.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 100); 
    pub_joint3 = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 100); 
    pub_joint4 = nh.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 100); 
    pub_joint5 = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 100); 
 
    // Register the signal handler for CTRL+C 
    signal(SIGINT, signalHandler); 
 
    ros::Rate loop_rate(10); // Control loop rate (10 Hz) 
 
    double frequency = 0.5;  // Frequency of the sine wave in Hz 
    double amplitude = 0.3;  // Amplitude of the sine wave 
    double phase_shift = 2* M_PI / 3; // Phase shift for each joint (in radians) 
    ros::Time start_time = ros::Time::now(); 
 
    while (ros::ok()) { 
        ros::Time current_time = ros::Time::now(); 
        double time = (current_time - start_time).toSec(); 
 
        // Create sine wave control signals for each joint 
        std_msgs::Float64 joint1_cmd; 
        std_msgs::Float64 joint2_cmd; 
        std_msgs::Float64 joint3_cmd; 
        
       
        std_msgs::Float64 joint4_cmd; 
        std_msgs::Float64 joint5_cmd; 
 
        // Alternate the direction for consecutive joints 
        joint1_cmd.data = -amplitude * sin(2.0 * M_PI * frequency * time); 
        joint2_cmd.data = amplitude * sin(2.0 * M_PI * frequency * time + phase_shift); 
        joint3_cmd.data = -amplitude * sin(2.0 * M_PI * frequency * time + 2*phase_shift); 
        joint4_cmd.data = amplitude * sin(2.0 * M_PI * frequency * time + 3*phase_shift); 
        joint5_cmd.data = -amplitude * sin(2.0 * M_PI * frequency * time + 4*phase_shift); 
 
        // Publish the commands to the joint topics 
        pub_joint1.publish(joint1_cmd); 
        pub_joint2.publish(joint2_cmd); 
        pub_joint3.publish(joint3_cmd); 
        pub_joint4.publish(joint4_cmd); 
        pub_joint5.publish(joint5_cmd); 
 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
 
    return 0; 
}
