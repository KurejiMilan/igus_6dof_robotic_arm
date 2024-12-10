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

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition_true = "True" if ros_distro == "humble" else "False"
    physics_engn = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("irc_ros_description"),
            "urdf",
            "igus_rebel_6dof.urdf.xacro"
        ]),
        description="Path to the .urdf.xacro file to use"
    )
    
    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition",
        default_value= is_ignition_true,
        description="This is used to set the correct gz controller"
    )

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), 
        " ",
        LaunchConfiguration("xacro_path"),
        " is_ignition:=",
        LaunchConfiguration("is_ignition")
        ]),
        value_type=str
        )

    # using this environmental variable we should communicate with gazebo where in the file system we should look for the urdf model and meshes for simulation
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value = PathJoinSubstitution([
            FindPackageShare("irc_ros_description")
        ])
    )
        # value = Path[
        #     str(Path(FindPackageShare("irc_ros_description")).parent.resolve())
        # ]
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time" : True
                     }]
    )



    # start empty gazebo simulation in empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                # "/opt/ros/humble/share/ros_gz_sim",
                "launch",
                "gz_sim.launch.py"])
            ),
        launch_arguments=[
            ("gz_args", f" -v 4 -r empty.sdf {physics_engn}")
            ]
    )
    gz_spwan_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "igus_6dof"
                   ]
    )

    # This node is used to connect gazebo msg(created by gazebo plugin/simulator) into standard ros2 message, so that ros2 can use topics form gazebo 
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]

    )

    return LaunchDescription([
        xacro_path_arg,
        is_ignition_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spwan_entity,
        gz_ros2_bridge
    ])