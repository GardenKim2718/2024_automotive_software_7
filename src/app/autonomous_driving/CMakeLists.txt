################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(autonomous_driving)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ad_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)  # For Eigen library
find_package(geometry_msgs REQUIRED)

## Add include directories
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface
    ${CMAKE_SOURCE_DIR}/../../bsw/system/bridge
    ${EIGEN3_INCLUDE_DIRS}  # Include Eigen headers
)

################################################################################
# LaneProcessing Node
################################################################################
add_executable(lane_processing_node
    src/lane_processing_node.cpp
)

ament_target_dependencies(lane_processing_node
    rclcpp
    ad_msgs
    geometry_msgs
    Eigen3
)

################################################################################
# AutonomousDriving Node
################################################################################
add_executable(autonomous_driving_node
    src/autonomous_driving_node.cpp
)

ament_target_dependencies(autonomous_driving_node
    rclcpp
    std_msgs
    ad_msgs
    tf2
    tf2_ros
    geometry_msgs
)

################################################################################
# Link libraries (if necessary)
################################################################################
target_link_libraries(lane_processing_node)
target_link_libraries(autonomous_driving_node)

################################################################################
# Install executables
################################################################################
install(TARGETS lane_processing_node autonomous_driving_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/simultaneous_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Package definition
################################################################################
ament_package()
