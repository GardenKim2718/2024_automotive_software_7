################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(ad_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

################################################################################
# Set message files
################################################################################
set(MSG_DEPS 
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
)

set(MSG_FILES
    # Vehicle
    msg/VehicleState.msg
    msg/VehicleCommand.msg
    
    # Missions
    msg/Mission.msg
    msg/MissionObject.msg
    msg/MissionRegion.msg
    msg/MissionDisplay.msg

    # Line
    msg/PolyfitLaneData.msg
    msg/LanePointData.msg
    msg/LanePointDataArray.msg
    msg/PolyfitLaneDataArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()