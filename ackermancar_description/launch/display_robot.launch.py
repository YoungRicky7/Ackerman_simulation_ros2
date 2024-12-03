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
    package_name = 'ackermancar_description'
    urdf_name = "hunter.urdf"
    file_prefix = "urdf/orbbec_hunter2/"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f"{file_prefix+urdf_name}")

    rviz_path = os.path.join(pkg_share ,'rviz/display_robot_model.rviz')

    rvizconfig = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_path),
        description="Absolute path to rviz config file",
    )

    robot_description = ParameterValue(Command(['cat'+' ', urdf_model_path]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        )

    return LaunchDescription([robot_state_publisher_node, joint_state_publisher_node, rvizconfig,rviz2_node])
