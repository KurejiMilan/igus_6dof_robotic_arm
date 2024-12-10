// code that demonstrates how to start with motion planning in  C++ and Moveit2

//necessary memory include
#include<memory>
// rclcpp-ROS Client library for C++ package
#include "rclcpp/rclcpp.hpp"
//this is necessary to plan and execute motion
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "geometry_msgs/msg/pose.h"

// for converting Euler angles to quaternions
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    bool use_sim = true;
    // create ros2 node
    auto const node = std::make_shared<rclcpp::Node>("inverse_path_solver");
    node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim));

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread spin_thread([executor]() { executor->spin(); });

    //create logger object
    auto const logger = rclcpp::get_logger("inverse_path_solver");
    
    // Create moveit interfaces
    moveit::planning_interface::MoveGroupInterface igus_6dof_arm(node, "rebel_6dof");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Publisher for visualizing the trajectory
    auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/move_group/display_planned_path", 1);

    // Log reference frames
    RCLCPP_INFO(node->get_logger(), "Reference frame: %s", igus_6dof_arm.getPlanningFrame().c_str());
    // RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    auto current_state = igus_6dof_arm.getCurrentState();
    if(!current_state){
        RCLCPP_ERROR(logger, "Failed to get current state! Ensure joint_states are being published.");
        rclcpp::shutdown();
        return -1;       
    }
    current_state->setToDefaultValues();
    igus_6dof_arm.setStartState(*current_state);
    
    // these are valid goal pose obtained using setRandomTarget()
    geometry_msgs::msg::Pose GoalPose;
    GoalPose.orientation.x = 0.000097;
    GoalPose.orientation.y = 0.442176;
    GoalPose.orientation.z = 0.000076;
    GoalPose.orientation.w = 0.896928;       

    GoalPose.position.x = 0.465228;
    GoalPose.position.y = -0.000061;
    GoalPose.position.z = 0.673601;

    igus_6dof_arm.setPlanningTime(10.0);
    // Change planner to RRTConnect
    igus_6dof_arm.setPlannerId("RRTConnectkConfigDefault");
    igus_6dof_arm.setPoseTarget(GoalPose);
    
    // the following lines of code are set up for debugging and understanding valid targets 
    // igus_6dof_arm.setRandomTarget();
    // geometry_msgs::msg::Pose random_pose = igus_6dof_arm.getCurrentPose().pose;
    // //printing valid pose
    // // Random Target Pose: Position(0.465228, -0.000061, 0.673601) Orientation(0.000097, 0.442176, 0.000076, 0.896928)
    // RCLCPP_INFO(rclcpp::get_logger("random_pose_logger"), "Random Target Pose: Position(%f, %f, %f) Orientation(%f, %f, %f, %f)", 
    //     random_pose.position.x, random_pose.position.y, random_pose.position.z,
    //     random_pose.orientation.x, random_pose.orientation.y,
    //     random_pose.orientation.z, random_pose.orientation.w);

    igus_6dof_arm.setGoalPositionTolerance(0.05);
    igus_6dof_arm.setGoalOrientationTolerance(0.1);
    
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool success = (igus_6dof_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);    
    // sleep(5.0);
    // Execute the plan
    if (success){
        // MoveGroupInterface.execute(arm_plan);
        RCLCPP_INFO(logger, "planned successful, executing...");
        igus_6dof_arm.execute(arm_plan);
    }
    else{
        RCLCPP_ERROR(logger, "Not able to plan and execute!");
    }

    rclcpp::shutdown();
    return 0;
}