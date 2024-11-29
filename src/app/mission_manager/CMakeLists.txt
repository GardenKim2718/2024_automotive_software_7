################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(mission_manager)

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

## Scenario Runner Node
add_executable(mission_manager
    # BSW
    ${CMAKE_SOURCE_DIR}/../../bsw/system/interface/interface_lane.cpp

    # Node and Algorithm
    src/mission_manager_node.cpp
)

################################################################################
# Link libraries and dependencies
################################################################################

# Define an algorithm library target for the pre-built algorithm.a
add_library(reference_lane_generation STATIC IMPORTED)
add_library(mission_merge STATIC IMPORTED)
add_library(mission_road_condition STATIC IMPORTED)
add_library(mission_road_slope STATIC IMPORTED)
add_library(mission_scc STATIC IMPORTED)
add_library(mission_speed_limit STATIC IMPORTED)

# Set the location of the algorithm library
set_target_properties(reference_lane_generation PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/reference_lane_generation.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(mission_merge PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/missions/mission_merge.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(mission_road_condition PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/missions/mission_road_condition.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(mission_road_slope PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/missions/mission_road_slope.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(mission_scc PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/missions/mission_scc.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set_target_properties(mission_speed_limit PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/src/missions/mission_speed_limit.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

## Scenario Runner Node
target_link_libraries(mission_manager
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    reference_lane_generation
    mission_merge
    mission_road_condition
    mission_road_slope
    mission_scc
    mission_speed_limit
)

ament_target_dependencies(mission_manager
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
    mission_manager
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
