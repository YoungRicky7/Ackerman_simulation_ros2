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
from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo

# obtain the "share" path of the package
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

# parameters load
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(name="use_sim_time", default_value="true")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ackermancar_description"),
        "launch",
        "gazebo.launch.py",
    )

    teleop_command = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "ros2",
            "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
        ],
        name="teleop_keyboard",
        output="screen",
    )

    teleop_command_remapped = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "ros2",
            "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            "--ros-args",
            "-r",
            "cmd_vel:=/orbbec_hunter2/cmd_vel",
        ],
        name="teleop_keyboard",
        output="screen",
    )

    gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch_file))
    return LaunchDescription(
        [use_sim_time, gazebo_launch, teleop_command]
    )
