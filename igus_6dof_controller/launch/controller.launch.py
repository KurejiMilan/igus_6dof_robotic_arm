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

def generate_launch_description():

    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("irc_ros_description"),
            "urdf",
            "igus_rebel_6dof.urdf.xacro"
        ]),
        description="Path to the .urdf.xacro file to use"
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
    # need one more for the gripper
    return LaunchDescription([
        xacro_path_arg,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        igus_6dof_arm_spawner
    ])