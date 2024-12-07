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
    robot_name = "hunter2"  # hunter hunter2
    file_prefix = "urdf/"+robot_name+"/"
    package_name = "ackermancar_description"
    world_file_path = "worlds/simple_world.world"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_path, world_file_path)  
    print(world_path)
    xacro_name = "hunter2.urdf.xacro"
    xacro_model_path = os.path.join(pkg_share, f"{file_prefix+xacro_name}")

    rviz_path = os.path.join(pkg_share, "rviz/gazebo.rviz")

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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # rviz2_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rvizconfig")],
    # )

    spawn_x_val = '-3.0'
    spawn_y_val = '-8.0'
    spawn_z_val = '0.4'
    spawn_yaw_val = '0.0'

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=[("world", world_path), ("verbose", "true")],
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                  arguments=['-topic', 'robot_description',
                              '-entity', robot_name,
                              '-x', spawn_x_val,
                              '-y', spawn_y_val,
                              '-z', spawn_z_val,
                              '-Y', spawn_yaw_val],
                  output='screen')

    return LaunchDescription(
        [
            use_sim_time,
            launch_gazebo,
            spawn_entity,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rvizconfig,
        ]
    )
