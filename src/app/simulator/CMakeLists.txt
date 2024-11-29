################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(simulator)

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
find_package(visualization_msgs REQUIRED)
find_package(ad_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)

## Add include directories
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface
    ${CMAKE_SOURCE_DIR}/../../bsw/system/bridge
)

## Declare a C++ executable
add_executable(vehicle 
    src/vehicle/vehicle_simulator_node.cpp
)
add_executable(lane_detection 
    # BSW
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface/interface_lane.cpp

    # Node and Algorithm
    src/lane_detection/lane_detection_node.cpp
)
add_executable(display 
    src/display/display_node.cpp
)


################################################################################
# Link libraries and dependencies
################################################################################

# Define an algorithm library target for the pre-built algorithm.a
add_library(vehicle_simulator_algorithm STATIC IMPORTED)
add_library(lane_detection_algorithm STATIC IMPORTED)

# Set the location of the algorithm library
set_target_properties(vehicle_simulator_algorithm PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle/vehicle_simulator_algorithm.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(lane_detection_algorithm PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection/lane_detection_algorithm.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

## Link libraries to the executable
target_link_libraries(vehicle
    vehicle_simulator_algorithm
)
target_link_libraries(lane_detection 
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    lane_detection_algorithm
)

## Add dependencies to the executable
ament_target_dependencies(vehicle
    rclcpp
    std_msgs
    ad_msgs
    tf2
    tf2_ros
)
ament_target_dependencies(lane_detection
    rclcpp
    std_msgs
    visualization_msgs
    ad_msgs
    tf2
    tf2_ros
)
ament_target_dependencies(display
    rclcpp
    std_msgs
    ad_msgs
    tf2
    tf2_ros
)

## Install the executable
install(TARGETS 
    vehicle
    lane_detection
    display
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME})

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
