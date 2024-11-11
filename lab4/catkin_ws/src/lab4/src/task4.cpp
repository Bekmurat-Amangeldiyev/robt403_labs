#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>  // Include for publishing positions

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // Publisher for visualizing positions in real-time
    ros::Publisher position_pub = node_handle.advertise<std_msgs::Float64MultiArray>("robot_position", 10);

    // Use an AsyncSpinner to process ROS callbacks
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Define the planning group for MoveIt
    static const std::string PLANNING_GROUP = "move_almat";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Get the initial current pose and define target poses
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose;

    // Move to the starting point O (12 o'clock position)
    current_pose = move_group.getCurrentPose();
    target_pose = current_pose;
    target_pose.pose.position.x -= 0.3;  // Move 0.3 along the -x direction to point O (12 o'clock position)
    move_group.setApproximateJointValueTarget(target_pose);
    move_group.move();

    // Define the center of the circle relative to point O (already at 12 o'clock)
    double center_x = target_pose.pose.position.x;
    double center_y = target_pose.pose.position.y;

    // Circle parameters
    double radius = 0.3;        // Circle radius
    int num_points = 36;        // Number of points to approximate the circle (smoother path)
    double angle_increment = -2 * M_PI / num_points;  // Clockwise angle increment (negative)

    // Loop through each point along the circle and move to that point
    for (int i = 0; i < num_points; ++i)
    {
        double angle = i * angle_increment;

        // Calculate the target position along the circle
        target_pose.pose.position.x = center_x + radius * cos(angle);  // No additional offset needed
        target_pose.pose.position.y = center_y + radius * sin(angle);

        // Publish the position for visualization
        std_msgs::Float64MultiArray position_msg;
        position_msg.data = {target_pose.pose.position.x, target_pose.pose.position.y};
        position_pub.publish(position_msg);

        // Set the approximate joint value target and move to the point
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();

        // Optional: Add a small delay if needed for smoother visualization
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Circle drawing complete.");
    ros::shutdown();
    return 0;
}

