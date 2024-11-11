// #include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// Main MoveIt libraries are included

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // For MoveIt implementation, we need AsyncSpinner, as we can't use ros::spinOnce()
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Specify the planning group for MoveIt
    static const std::string PLANNING_GROUP = "move_almat";  // group1 is the name of the group controller
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // Load move_group

    // For joint control
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Define current and target poses
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose;

    // Retrieve the current position and orientation of the end effector
    current_pose = move_group.getCurrentPose();
    target_pose = current_pose;

    // Set target pose to move 0.1 along the x-axis
    target_pose.pose.position.x = target_pose.pose.position.x - 1.4;

    ros::Rate loop_rate(50);  // Frequency

    while (ros::ok())
    {
        // Set approximate joint value target for the specified pose
        move_group.setApproximateJointValueTarget(target_pose);

        // Move the robot
        move_group.move();

        // Update the current pose
        current_pose = move_group.getCurrentPose();

        // Check if the desired position is reached
        if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01)
        {
            break;
        }
        loop_rate.sleep();
    }

    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}

