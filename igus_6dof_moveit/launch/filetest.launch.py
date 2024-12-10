# import LaunchDescription class
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import  Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import TimerAction

def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder("igus_rebel_6dof", package_name="igus_6dof_moveit")
    #     .robot_description(file_path="config/igus_rebel_6dof.urdf.xacro")
    #     .robot_description_semantic(file_path="config/igus_rebel_6dof.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )
    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("irc_ros_description"),
            "urdf",
            "igus_rebel_6dof.urdf.xacro"
        ]),
        description="Path to the .urdf.xacro file to use"
    )

    moveit_config = (
        MoveItConfigsBuilder("igus_rebel_6dof", package_name="igus_6dof_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("igus_6dof_moveit"),
            "config",
            "igus_rebel_6dof.urdf.xacro"
        ))
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory("igus_6dof_moveit"),
            "config",
            "igus_rebel_6dof.srdf"
        ))
        .trajectory_execution(file_path=os.path.join(
            get_package_share_directory("igus_6dof_moveit"),
            "config",
            "moveit_controllers.yaml"
        ))
        .to_moveit_configs()
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("igus_6dof_moveit"),
        "config",
        "igus_controllers.yaml",
    )

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), 
        " ",
        LaunchConfiguration("xacro_path"),
        # " is_ignition:=",
        # LaunchConfiguration("is_ignition")
        ]),
        value_type=str
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time" : True
                     }]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    igus_6dof_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    move_group_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output = "screen",
                parameters=[moveit_config.to_dict(), 
                            {"use_sim_time": True}, 
                            {"publish_robot_description_semantic":True}],
                arguments=["--ros-args", "--log-level", "info"]
            )
        ]
    )

    # need one more for the gripper
    return LaunchDescription([
        xacro_path_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        igus_6dof_arm_spawner,
        move_group_node
    ])
