cmake_minimum_required(VERSION 3.5)
project(kr_example_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kr_msgs REQUIRED)

add_executable(robot_state src/robot_state.cpp)
add_executable(follow_joint src/follow_joint.cpp)
add_executable(move_joint src/move_joint.cpp)
add_executable(jog_linear src/jog_linear.cpp)
add_executable(self_motion src/self_motion.cpp)

ament_target_dependencies(robot_state rclcpp std_msgs kr_msgs)
ament_target_dependencies(follow_joint rclcpp std_msgs kr_msgs)
ament_target_dependencies(move_joint rclcpp std_msgs kr_msgs)
ament_target_dependencies(jog_linear rclcpp std_msgs kr_msgs)
ament_target_dependencies(self_motion rclcpp std_msgs kr_msgs)

install(TARGETS
  robot_state
  follow_joint
  move_joint
  jog_linear
  self_motion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
