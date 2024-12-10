#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

// commenting out gripper parts for now

void  move_robot(const std::shared_ptr<rclcpp::Node> node) {
  auto igus_6dof_arm = moveit::planning_interface::MoveGroupInterface(node, "rebel_6dof");
  // auto gripper = moveit::planning_interface::MoveGroupInterface(node, "igus_gripper");
  std::vector<double> arm_joint_goal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // std::vector<double> gripper_joint_goal {-0.7, 0.7};

  bool arm_within_bounds = igus_6dof_arm.setJointValueTarget(arm_joint_goal);
  //bool gripper_within_bounds  gripper.setJointValueTarget(gripper_joint_goal);
  if(!arm_within_bounds){
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position were outside of limits!");
    return;
  }
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
  // moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
  bool arm_plan_success = igus_6dof_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  //gripper_plan_success = gripper.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  if(arm_plan_success){
    igus_6dof_arm.move();
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed");
    return;
  }
}
int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
  // take input node as an argument
  move_robot(node);
  rclcpp::shutdown();
  return 0;
}