cmake_minimum_required(VERSION 3.5)
project(ur5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)
# Add executable
add_executable(position_publisher src/position_publisher.cpp)
add_executable(moveit_cartesian_planner src/moveit_cartesian_planner.cpp)
add_executable(position_pub_nomoveit src/position_pub_nomoveit.cpp)



# Link libraries
ament_target_dependencies(position_publisher
  rclcpp
  std_msgs
  kdl_parser
  orocos_kdl
  urdf
  ament_index_cpp
)

ament_target_dependencies(position_pub_nomoveit
  rclcpp
  std_msgs
  sensor_msgs
  kdl_parser
  orocos_kdl
  urdf
  ament_index_cpp
)


ament_target_dependencies(moveit_cartesian_planner
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
)


# Install targets
install(TARGETS
  position_pub_nomoveit
  position_publisher
  moveit_cartesian_planner
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
