cmake_minimum_required(VERSION 3.8)
project(ackermancar_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(amcl_auto_init src/amcl_auto_init.cpp)
ament_target_dependencies(
  amcl_auto_init 
  "gazebo_msgs"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )

install(TARGETS amcl_auto_init
  DESTINATION lib/${PROJECT_NAME})

add_executable(compute_tracking_time src/compute_tracking_time.cpp)
ament_target_dependencies(
  compute_tracking_time
  "gazebo_msgs"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "ackermann_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )

install(TARGETS compute_tracking_time
  DESTINATION lib/${PROJECT_NAME})

add_executable(set_goal_pose src/set_goal_pose.cpp)
ament_target_dependencies(
  set_goal_pose
  "gazebo_msgs"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )

install(TARGETS set_goal_pose
  DESTINATION lib/${PROJECT_NAME})

add_executable(smoother_client src/smoother_client.cpp)
ament_target_dependencies(
  smoother_client
  "gazebo_msgs"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )

install(TARGETS smoother_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(visualize_path src/visualize_path.cpp)
ament_target_dependencies(
  visualize_path
  "gazebo_msgs"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "nav2_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )
install(TARGETS visualize_path
  DESTINATION lib/${PROJECT_NAME})
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params rviz maps DESTINATION share/${PROJECT_NAME})

ament_package()
