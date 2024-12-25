from launch import LaunchDescription
from launch_ros.actions import Node

# terminator command class
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

# parameters' declare and obtain in a node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# file, use launch to launch muitiple launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# group
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

# event
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo

# obtain the "share" path of the package
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

# parameters load
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ackermancar_description"),
        "launch",
        "gazebo_with_teleop_twist_keyboard.launch.py",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("ackermancar_navigation2"),
            "params",
            "slam_toolbox",
            "mapper_params_online_async.yaml",
        ),
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    return LaunchDescription([gazebo_launch,use_sim_time,slam_params_file,start_async_slam_toolbox_node])
