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
    robot_name = "neor_mini"
    map_name = "simple_world"
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ackermancar_description"),
        "launch",
        "gazebo.launch.py",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    pkg_share = get_package_share_directory("ackermancar_navigation2")
    nav2_share = get_package_share_directory("nav2_bringup")

    rviz_file_path = os.path.join(
        pkg_share,
        "rviz",
        "nav2_default_view.rviz",
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file_path]
    )

    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_yaml_path = DeclareLaunchArgument(
        "map_yaml_path", default_value=os.path.join(pkg_share, "maps", map_name+".yaml")
    )
    param_path = DeclareLaunchArgument(
        "params_file_path",
        default_value=os.path.join(pkg_share, "params"+"/nav2", robot_name+"_nav2_params.yaml"),
    )

    nav2_bringup_path = os.path.join(
        nav2_share,
        'launch',
        'bringup_launch.py'
    )
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            "map": LaunchConfiguration("map_yaml_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file_path"),
        }.items(),
    )

    # x: -2.9, y: -7.99 for simple_word
    # x: -12.0, y: 19.5 for simple_race_road
    # initial_pose_cmd = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ros2"),
    #         "topic",
    #         "pub",
    #         "-1",
    #         "/initialpose",
    #         "geometry_msgs/PoseWithCovarianceStamped",
    #         "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -12.0, y: 19.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}",
    #     ],
    #     output="screen",
    # )

    amcl_init = Node(
        package='ackermancar_navigation2',
        executable='amcl_auto_init',
        output='screen',
    )

    return LaunchDescription(
        [
            use_sim_time,
            map_yaml_path,
            param_path,
            gazebo_launch,
            nav2_bringup_launch,
            rviz2_node,
            amcl_init,
        ]
    )
