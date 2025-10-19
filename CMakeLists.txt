cmake_minimum_required(VERSION 3.8)
project(rosbag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check ROS2 distribution
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  set(ROS_DISTRO "humble")  # Default to humble
endif()

message(STATUS "Building for ROS2 ${ROS_DISTRO}")

# Add compile definitions based on ROS distro
if(${ROS_DISTRO} STREQUAL "jazzy")
  add_compile_definitions(ROS_DISTRO_JAZZY)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

# find dependencies using ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RosbagRecorderStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# Include directories
include_directories(include)

# Create rosbag recorder library
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/rosbag_recorder_node.cpp
)

# Link generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# Register the component
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "rosbag_recorder::RosbagRecorderNode")

# Create executable
ament_auto_add_executable(rosbag_recorder_node src/main.cpp)
target_link_libraries(rosbag_recorder_node ${PROJECT_NAME}_lib)

# Install config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
