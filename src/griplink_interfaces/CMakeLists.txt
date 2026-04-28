cmake_minimum_required(VERSION 3.8)
project(griplink_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------------------------
# | GriplinkNode-ROS2-Interfaces
# -----------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}

  # Messages
  msg/DeviceStates.msg
  
  # Services
  srv/Id.srv
  srv/Protocol.srv
  srv/Protassert.srv
  srv/Sn.srv
  srv/Labelget.srv
  srv/Labelset.srv
  srv/Ver.srv
  srv/Verbose.srv
  srv/Devvid.srv
  srv/Devpid.srv
  srv/Devassert.srv
  srv/Devname.srv
  srv/Devvendor.srv
  srv/Devsn.srv
  srv/Devtagget.srv
  srv/Devtagset.srv
  srv/Devver.srv
  srv/Bye.srv
  srv/Enable.srv
  srv/Disable.srv
  srv/Home.srv
  srv/Grip.srv
  srv/Release.srv
  srv/Flexgrip.srv
  srv/Flexrelease.srv
  srv/Led.srv
  srv/Clamp.srv
  srv/Wstr.srv
  srv/Setval.srv
  srv/Waitval.srv
  srv/Devstate.srv
  srv/Value.srv
  srv/Gripcfgget.srv
  srv/Gripcfgset.srv

  # Actions
  action/Grip.action
  action/Release.action  
  action/Flexgrip.action
  action/Flexrelease.action
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