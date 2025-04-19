cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)



include_directories(
  ${moveit_core_INCLUDE_DIRS}  # Ensure MoveIt core includes are added
  ${moveit_ros_planning_INCLUDE_DIRS}
)

# add_executable(test_node src/test.cpp)
# ament_target_dependencies(test_node rclcpp std_msgs trajectory_msgs)

#add_executable(mtc_test src/mtc_node.cpp)

add_executable(mtc_test src/mtc_moveit.cpp)
ament_target_dependencies(mtc_test rclcpp std_msgs trajectory_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface)

add_executable(ur3_control_node src/ur3_control.cpp)
ament_target_dependencies(ur3_control_node rclcpp std_msgs trajectory_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface)

add_executable(pose_array_publisher src/test_publish_node.cpp)
ament_target_dependencies(pose_array_publisher rclcpp geometry_msgs)


# add_executable(mtc_main src/main.cpp src/mtc_task.cpp)
# ament_target_dependencies(mtc_main rclcpp std_msgs trajectory_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_task_constructor_core moveit_task_constructor_msgs)

install(TARGETS
  #test_node
  mtc_test
  ur3_control_node
  pose_array_publisher
  # mtc_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  #launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
