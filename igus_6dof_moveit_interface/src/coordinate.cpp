if (success) {
    RCLCPP_INFO(logger, "Plan was successful, printing Cartesian positions...");

    // Extract the trajectory
    const auto& trajectory = arm_plan.trajectory_.joint_trajectory;

    // Get the robot model
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");
    auto robot_model = robot_model_loader->getModel();
    if (!robot_model) {
        RCLCPP_ERROR(logger, "Failed to load robot model!");
        return -1;
    }

    // Create a RobotState object
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();
    
    // Iterate over trajectory points
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        RCLCPP_INFO(logger, "Point %lu:", i);

        // Set the joint positions for the robot state
        robot_state.setJointGroupPositions(igus_6dof_arm.getName(), trajectory.points[i].positions);

        // Update FK to calculate the end-effector pose
        const auto& end_effector_link = igus_6dof_arm.getEndEffectorLink();
        const Eigen::Isometry3d& ee_pose = robot_state.getGlobalLinkTransform(end_effector_link);

        // Extract and print Cartesian coordinates
        RCLCPP_INFO(logger, "  End-effector position: [x: %f, y: %f, z: %f]",
                    ee_pose.translation().x(),
                    ee_pose.translation().y(),
                    ee_pose.translation().z());
    }
} else {
    RCLCPP_ERROR(logger, "Planning failed!");
}
