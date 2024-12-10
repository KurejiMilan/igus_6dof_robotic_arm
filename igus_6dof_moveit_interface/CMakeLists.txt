cmake_minimum_required(VERSION 3.8)
project(igus_6dof_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(forward_path_planner src/forward_path_planner.cpp)
ament_target_dependencies(forward_path_planner rclcpp moveit_ros_planning_interface)

add_executable(inverse_path_solver src/inverse_path_solver.cpp)
ament_target_dependencies(inverse_path_solver
  rclcpp 
  moveit_ros_planning_interface 
  geometry_msgs 
  tf2_geometry_msgs 
  tf2
  moveit_core
  moveit_msgs
)

install(TARGETS
  forward_path_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  inverse_path_solver
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
