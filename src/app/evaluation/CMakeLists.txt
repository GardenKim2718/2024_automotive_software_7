################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(evaluation)

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
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)

## Add include directories
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface
    ${CMAKE_SOURCE_DIR}/../../bsw/system/bridge
)

################################################################################
# Add executables
################################################################################

## Evaluation Node
add_executable(evaluation
    # BSW
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface/interface_lane.cpp

    # Node and Algorithm
    src/evaluation_node.cpp
)

################################################################################
# Link libraries and dependencies
################################################################################

# Define an algorithm library target for the pre-built algorithm.a
add_library(evaluation_algorithm STATIC IMPORTED)

# Set the location of the algorithm library
set_target_properties(evaluation_algorithm PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/evaluation_algorithm.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

## Evaluation Node
target_link_libraries(evaluation
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    evaluation_algorithm
)

ament_target_dependencies(evaluation
    rclcpp
    std_msgs
    ad_msgs
    rviz_2d_overlay_msgs
    tf2
    tf2_ros
)

################################################################################
# Install the executables
################################################################################

install(TARGETS 
    evaluation
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

################################################################################
# ament package
################################################################################
ament_package()
