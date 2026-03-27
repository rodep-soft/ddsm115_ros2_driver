cmake_minimum_required(VERSION 3.8)
project(ddsm115_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(DEFINED ENV{CONDA_PREFIX})
  include_directories(SYSTEM "$ENV{CONDA_PREFIX}/include")
  string(APPEND CMAKE_CXX_FLAGS " -isystem $ENV{CONDA_PREFIX}/include")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED CONFIG COMPONENTS headers)
find_package(fastcdr REQUIRED CONFIG)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/Ddsm115Command.msg"
  "msg/Ddsm115Status.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Add executable
add_executable(ddsm115_driver_node
  src/ddsm115_ros2_driver_node.cpp
  src/ddsm115_ros2_driver_client.cpp
)

target_include_directories(ddsm115_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ddsm115_driver_node PUBLIC cxx_std_20)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(ddsm115_driver_node PUBLIC
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  Boost::headers
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  "${cpp_typesupport_target}"
)

# # Install targets
install(TARGETS
  ddsm115_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts files
install(PROGRAMS
  scripts/set_motor_id.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
