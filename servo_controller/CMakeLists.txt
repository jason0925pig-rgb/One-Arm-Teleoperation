cmake_minimum_required(VERSION 3.8)
project(servo_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(MODBUS_LIBRARY modbus REQUIRED)
find_path(MODBUS_INCLUDE_DIR modbus/modbus.h REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JointMove.srv"
  DEPENDENCIES geometry_msgs
)

# Select the supplied JAKA SDK that matches the Ubuntu computer architecture.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|arm")
  set(ARCH_DIR "arm")
  set(JAKA_LIB_PATH "${PROJECT_SOURCE_DIR}/lib/${ARCH_DIR}/libjakaAPI.a")
  add_compile_definitions(ARCH_ARM64=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(ARCH_DIR "x86_64_new")
  set(JAKA_LIB_PATH "${PROJECT_SOURCE_DIR}/lib/${ARCH_DIR}/libjakaAPI.so")
  add_compile_definitions(ARCH_X86_64=1)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS "${JAKA_LIB_PATH}")
  message(FATAL_ERROR "JAKA SDK library is missing: ${JAKA_LIB_PATH}")
endif()

# New safety-gated executables. They do not power or enable real hardware at
# startup, and real motion remains locked until explicit parameters/services.
add_executable(safe_one_arm_servo
  src/safe_one_arm_servo.cpp
  src/robot.cpp
)
target_include_directories(safe_one_arm_servo PUBLIC
  include
  "include/${ARCH_DIR}"
)
target_link_libraries(safe_one_arm_servo "${JAKA_LIB_PATH}" Threads::Threads)
ament_target_dependencies(safe_one_arm_servo
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(safety_slew_limiter_smoke
    test/safety_slew_limiter_smoke.cpp
  )
  target_include_directories(safety_slew_limiter_smoke PRIVATE include)
  add_test(
    NAME safety_slew_limiter_smoke
    COMMAND safety_slew_limiter_smoke
  )
endif()

add_executable(safe_gripper_controller
  src/safe_gripper_controller.cpp
  src/DH_gripper.cpp
  src/ZX_gripper.cpp
)
target_include_directories(safe_gripper_controller PUBLIC
  include
  "${MODBUS_INCLUDE_DIR}"
)
target_link_libraries(safe_gripper_controller "${MODBUS_LIBRARY}")
ament_target_dependencies(safe_gripper_controller
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

# Read-only feedback probe used to capture operator-chosen safe gripper poses.
add_executable(gripper_feedback_probe
  src/gripper_feedback_probe.cpp
  src/DH_gripper.cpp
  src/ZX_gripper.cpp
)
target_include_directories(gripper_feedback_probe PUBLIC
  include
  "${MODBUS_INCLUDE_DIR}"
)
target_link_libraries(gripper_feedback_probe "${MODBUS_LIBRARY}")
ament_target_dependencies(gripper_feedback_probe
  rclcpp
  sensor_msgs
  std_msgs
)

# Teacher-supplied programs can power/enable hardware and are intentionally
# excluded from normal builds. They remain in src/ for reference.
option(BUILD_LEGACY_DUAL_ARM_CONTROLLER
  "Build the unsafe legacy robot_timer executable" OFF)
option(BUILD_LEGACY_TEST_EXECUTABLES
  "Build teacher-supplied legacy test executables" OFF)

if(BUILD_LEGACY_DUAL_ARM_CONTROLLER OR BUILD_LEGACY_TEST_EXECUTABLES)
  find_package(trajectory_msgs REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(yaml-cpp REQUIRED)
endif()

if(BUILD_LEGACY_DUAL_ARM_CONTROLLER)
  add_executable(robot_timer
    src/servo_controller.cpp
    src/trajectory_online_interpolator.cpp
    src/hand_control.cpp
    src/robot.cpp
    src/DH_gripper.cpp
    src/ZX_gripper.cpp
  )
  target_include_directories(robot_timer PUBLIC
    include
    "include/${ARCH_DIR}"
    include/servo_controller
    "${MODBUS_INCLUDE_DIR}"
  )
  target_link_libraries(robot_timer
    "${JAKA_LIB_PATH}"
    Eigen3::Eigen
    "${MODBUS_LIBRARY}"
    yaml-cpp
  )
  ament_target_dependencies(robot_timer
    rclcpp
    std_srvs
    geometry_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
  )
  rosidl_target_interfaces(robot_timer
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS robot_timer DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_LEGACY_TEST_EXECUTABLES)
  add_executable(test_joint_trajectory_sub
    src/test_joint_trajectory_sub.cpp
    src/robot.cpp
    src/trajectory_online_interpolator.cpp
    src/hand_control.cpp
    src/DH_gripper.cpp
    src/ZX_gripper.cpp
  )
  target_include_directories(test_joint_trajectory_sub PUBLIC
    include
    "include/${ARCH_DIR}"
    include/servo_controller
    "${MODBUS_INCLUDE_DIR}"
  )
  target_link_libraries(test_joint_trajectory_sub
    "${JAKA_LIB_PATH}"
    Eigen3::Eigen
    "${MODBUS_LIBRARY}"
    yaml-cpp
  )
  ament_target_dependencies(test_joint_trajectory_sub
    rclcpp
    std_srvs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
  )
  rosidl_target_interfaces(test_joint_trajectory_sub
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS test_joint_trajectory_sub DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  safe_one_arm_servo
  safe_gripper_controller
  gripper_feedback_probe
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

# The x86 SDK is dynamically linked. Put the selected runtime in the regular
# ROS package lib directory and teach only our executable where to find it.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  install(FILES "${JAKA_LIB_PATH}" DESTINATION lib)
  set_target_properties(safe_one_arm_servo PROPERTIES
    INSTALL_RPATH "\$ORIGIN/.."
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
