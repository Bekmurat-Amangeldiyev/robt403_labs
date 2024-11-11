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
    // Initializing a ROS Node 
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

    // Define a helper function to move to target pose
    auto moveToPose = [&move_group, &current_pose](geometry_msgs::PoseStamped& target_pose, double x, double y, double tolerance = 0.01) {
        target_pose.pose.position.x = current_pose.pose.position.x + x;
        target_pose.pose.position.y = current_pose.pose.position.y + y;
        
        ros::Rate loop_rate(50);  // Frequency

        while (ros::ok()) {
            move_group.setApproximateJointValueTarget(target_pose); // Set target
            move_group.move(); // Move the robot
            current_pose = move_group.getCurrentPose(); // Update current pose

            // Check if the desired position is reached
            if (std::abs(current_pose.pose.position.x - target_pose.pose.position.x) < tolerance &&
                std::abs(current_pose.pose.position.y - target_pose.pose.position.y) < tolerance) {
                break;
            }
            loop_rate.sleep();
        }
    };

    // Move in a rectangular path
    moveToPose(target_pose, -0.3, 0.0);   // Move to Point O (-0.3 along x)
    moveToPose(target_pose, 0.0, 0.3);    // Move to Point A (+0.3 along y)
    moveToPose(target_pose, -0.5, 0.0);   // Move to Point B (-0.5 along x)
    moveToPose(target_pose, 0.0, -0.6);   // Move to Point C (-0.6 along y)
    moveToPose(target_pose, 0.5, 0.0);    // Move to Point D (+0.5 along x)
    moveToPose(target_pose, 0.0, 0.3);    // Return to Point O (+0.3 along y)

    ROS_INFO("Rectangle path completed");
    ros::shutdown();
    return 0;
}

