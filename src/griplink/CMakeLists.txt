cmake_minimum_required(VERSION 3.8)
project(griplink)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(griplink_interfaces REQUIRED)


# -----------------------------------------------
# | Griplink-Node
# -----------------------------------------------

add_executable(
  griplink_node
	src/griplink_node/main.cpp
	src/griplink_node/griplink_node.cpp
	src/griplink_node/griplink.cpp
	src/griplink_node/common.cpp
)

ament_target_dependencies(
  griplink_node
	griplink_interfaces
	rclcpp
  rclcpp_action
)

target_compile_features(
  griplink_node 
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17

target_include_directories(
  griplink_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
	TARGETS
		griplink_node
	DESTINATION
		lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    include/
  DESTINATION 
    include
)


# -----------------------------------------------
# | Demo-Node
# -----------------------------------------------

add_executable(
  demo_node
	src/demo_node/main.cpp
	src/demo_node/demo_node.cpp
  src/griplink_node/common.cpp
)

ament_target_dependencies(
  demo_node
	griplink_interfaces
	rclcpp
  rclcpp_action
)

target_compile_features(
  demo_node 
  PUBLIC 
    c_std_99 
    cxx_std_17
)  # Require C99 and C++17

target_include_directories(
  demo_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
	TARGETS
    demo_node
	DESTINATION
		lib/${PROJECT_NAME})

# -----------------------------------------------
# | Testing
# -----------------------------------------------

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

# -----------------------------------------------
# | Misc installs & exports
# -----------------------------------------------

install(
  DIRECTORY 
    launch
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
	rosidl_default_runtime
)

ament_package()
