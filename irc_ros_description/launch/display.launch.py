# import LaunchDescription class
from launch import LaunchDescription
from launch_ros.actions import  Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # model_arg = DeclareLaunchArgument(name="model", 
    #                                   default_value=os.path.join(get_package_share_directory("irc_ros_description"), "urdf","igus_rebel_6dof.urdf.xacro"),
    #                                   description="Absolute path to the Robot URDF file"
    #                                   )
    
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
        LaunchConfiguration("xacro_path")])
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    joint_state_publisher_gui = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name = "joint_state_publisher"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2", 
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("irc_ros_description"), "rviz","rebel.rviz")]
    )
    # returns LaunchDescription object with all the nodes that are to be started
    return LaunchDescription([
        xacro_path_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])