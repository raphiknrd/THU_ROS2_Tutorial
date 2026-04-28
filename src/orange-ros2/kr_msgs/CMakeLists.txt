cmake_minimum_required(VERSION 3.5)
project(kr_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/FollowJoint.msg"
    "msg/FollowLinear.msg"
    "msg/GeometryPlane.msg"
    "msg/JogJoint.msg"
    "msg/JogLinear.msg"
    "msg/MotionHeartbeat.msg"
    "msg/Number.msg"
    "msg/ProgramInteract.msg"
    "msg/Quadrature.msg"
    "msg/RobotMode.msg"
    "msg/RobotPose.msg"
    "msg/RobotState.msg"
    "msg/SafetyEvent.msg"
    "msg/SafetyMode.msg"
    "msg/SafetyZone.msg" 
    "msg/SelfMotion.msg"
    "msg/SystemState.msg" 
    
    "srv/io/ConfigAnalogOutput.srv"
    "srv/io/ConfigDiscreteOutput.srv"
    "srv/io/ConfigQuadratureInput.srv"
    "srv/io/GetAnalogInput.srv"
    "srv/io/GetDiscreteInput.srv"
    "srv/io/GetQuadratureInput.srv"
    "srv/io/SetAnalogOutput.srv"
    "srv/io/SetDiscreteOutput.srv"
    
    "srv/motion/MoveJoint.srv"
    "srv/motion/MoveLinear.srv"
    "srv/motion/MoveSpline.srv"
    "srv/motion/PauseMotion.srv"
    "srv/motion/ResumeMotion.srv"
    "srv/motion/SelectJoggingFrame.srv"
    "srv/motion/SetCustomFrame.srv"
    "srv/motion/SetInteractivity.srv"
    "srv/motion/TerminateMotion.srv"
    
    "srv/program/GetProgramState.srv"
    "srv/program/PauseProgram.srv"
    "srv/program/ProgramRecovery.srv"
    "srv/program/ResumeProgram.srv"
    "srv/program/StepProgram.srv"
    "srv/program/StopProgram.srv"

    "srv/robot/GetRobotPose.srv"
    "srv/robot/GetSystemFrame.srv"
    "srv/robot/GetSystemLoad.srv"
    "srv/robot/SetSystemFrame.srv"
    "srv/robot/SetSystemLoad.srv"
    
    "srv/system/GetMasterSpeed.srv"
    "srv/system/GetRobotMode.srv"
    "srv/system/GetRobotState.srv"
    "srv/system/GetSafetyMode.srv"
    "srv/system/GetSafetyZones.srv"
    "srv/system/LogAlarm.srv"
    "srv/system/SuspendRobot.srv"
    "srv/system/UnlockRobot.srv"
    "srv/system/UnsuspendRobot.srv"
    
    DEPENDENCIES builtin_interfaces rosidl_default_generators rosidl_default_runtime std_msgs unique_identifier_msgs
)



ament_package()
