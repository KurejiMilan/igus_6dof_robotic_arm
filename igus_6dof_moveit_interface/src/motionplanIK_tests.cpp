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
    // current_state->setToDefaultValues();
    igus_6dof_arm.setStartState(*current_state);
    
    // these are valid goal pose obtained using setRandomTarget()
    geometry_msgs::msg::Pose GoalPose;
    GoalPose.orientation.x = -0.000104;
    GoalPose.orientation.y = 0.442271;
    GoalPose.orientation.z = -0.000127;
    GoalPose.orientation.w = 0.896881;       

    GoalPose.position.x = 0.465300;
    GoalPose.position.y = 0.000110;
    GoalPose.position.z = 0.673511;

    igus_6dof_arm.setPlanningTime(10.0);
    // Change planner to RRTConnect
    igus_6dof_arm.setPlannerId("RRTConnectkConfigDefault");
    igus_6dof_arm.setPoseTarget(GoalPose);

    // igus_6dof_arm.setRandomTarget();
    // geometry_msgs::msg::Pose random_pose = igus_6dof_arm.getCurrentPose().pose;

    igus_6dof_arm.setGoalPositionTolerance(0.05);
    igus_6dof_arm.setGoalOrientationTolerance(0.1);
    
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool success = (igus_6dof_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);    
    // sleep(5.0);
    // Execute the plan
    if (success){
        // MoveGroupInterface.execute(arm_plan);
        RCLCPP_INFO(logger, "planned successful, printing trajectory...");

        //Extract the trajectory
        const auto& trajectory = arm_plan.trajectory_.joint_trajectory;
        
        RCLCPP_INFO(logger, "Trajectory contains %lu points", trajectory.points.size());
        for(size_t i =0; i<trajectory.points.size(); i++){
            RCLCPP_INFO(logger, "Point %lu:", i);
            // Print time from start
            RCLCPP_INFO(logger, "  Time from start: %f seconds", trajectory.points[i].time_from_start.sec + 1e-9 * trajectory.points[i].time_from_start.nanosec);
            // Print joint positions
            for (size_t j = 0; j < trajectory.points[i].positions.size(); j++) {
                RCLCPP_INFO(logger, "  Joint %lu position: %f", j, trajectory.points[i].positions[j]);
            }
        }
        // just logging not executing in this node.
        // igus_6dof_arm.execute(arm_plan);
    }
    else{
        RCLCPP_ERROR(logger, "Not able to plan and execute!");
    }
 
    rclcpp::shutdown();
    return 0;
}