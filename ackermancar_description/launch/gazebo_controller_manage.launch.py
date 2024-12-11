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

from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    robot_name = "hunter2"  # hunter hunter2
    file_prefix = "urdf/" + robot_name + "/"
    package_name = "ackermancar_description"
    world_file_path = "worlds/simple_world.world"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)
    print(world_path)
    xacro_name = "hunter2.urdf.xacro"
    xacro_model_path = os.path.join(pkg_path, f"{file_prefix+xacro_name}")

    rviz_path = os.path.join(pkg_path, "rviz/gazebo.rviz")

    rvizconfig = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_path),
        description="Absolute path to rviz config file",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    remap_odom_tf = DeclareLaunchArgument(
        "remap_odometry_tf",
        default_value="true",
        description="Remap odometry TF from the steering controller to the TF tree.",
    )

    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    robot_description = ParameterValue(
        Command(["xacro" + " ", xacro_model_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    spawn_x_val = "-3.0"
    spawn_y_val = "-8.0"
    spawn_z_val = "0.4"
    spawn_yaw_val = "0.0"

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=[("world", world_path), ("verbose", "true")],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name,
            "-x",
            spawn_x_val,
            "-y",
            spawn_y_val,
            "-z",
            spawn_z_val,
            "-Y",
            spawn_yaw_val,
        ],
        output="screen",
    )

    robot_controllers = os.path.join(pkg_path, "params", robot_name, "controller.yaml")

    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/bicycle_steering_controller/tf_odometry", "/tf"),
        ],
        condition=IfCondition(remap_odometry_tf),
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(remap_odometry_tf),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            use_sim_time,
            remap_odom_tf,
            launch_gazebo,
            spawn_entity,
            control_node_remapped,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
            rvizconfig,
        ]
    )
