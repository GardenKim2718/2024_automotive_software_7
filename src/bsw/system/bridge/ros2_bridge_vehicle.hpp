/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      ros2_bridge_vehicle.hpp
 * @brief     ROS2 bridge for vehicle
 * 
 * @date      2024-11-06 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 *  
 */

#ifndef __ROS2_BRIDGE_VEHICLE_HPP__
#define __ROS2_BRIDGE_VEHICLE_HPP__
#pragma once

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header
#include <ad_msgs/msg/vehicle_command.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// TF Header
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// Interface Header
#include "interface_vehicle.hpp"

namespace ros2_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline interface::VehicleState GetVehicleState(const ad_msgs::msg::VehicleState& msg) {
        interface::VehicleState vehicle_state;
        vehicle_state.id = msg.id;
        vehicle_state.x = msg.x;
        vehicle_state.y = msg.y;
        vehicle_state.pitch = msg.pitch;
        vehicle_state.yaw = msg.yaw;
        vehicle_state.yaw_rate = msg.yaw_rate;
        vehicle_state.slip_angle = msg.slip_angle;
        vehicle_state.velocity = msg.velocity;
        vehicle_state.length = msg.length;
        vehicle_state.width = msg.width;
        
        return vehicle_state;
    }
    
    inline interface::VehicleCommand GetVehicleCommand(const ad_msgs::msg::VehicleCommand& msg) {
        interface::VehicleCommand vehicle_command;
        vehicle_command.accel    = std::max(0.0, std::min(1.0, msg.accel));
        vehicle_command.brake    = std::max(0.0, std::min(1.0, msg.brake));
        vehicle_command.steering = std::max(-interface::vehicle_param::max_steer, 
                                   std::min(interface::vehicle_param::max_steer, msg.steering));     
        
        return vehicle_command;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline ad_msgs::msg::VehicleState UpdateVehicleState(const interface::VehicleState& vehicle_state) {
        ad_msgs::msg::VehicleState msg;
        msg.id = vehicle_state.id;
        msg.x = vehicle_state.x;
        msg.y = vehicle_state.y;
        msg.pitch = vehicle_state.pitch;
        msg.yaw = vehicle_state.yaw;
        msg.yaw_rate = vehicle_state.yaw_rate;
        msg.slip_angle = vehicle_state.slip_angle;
        msg.velocity = vehicle_state.velocity;
        msg.length = vehicle_state.length;
        msg.width = vehicle_state.width;
        
        return msg;
    }

    inline geometry_msgs::msg::TransformStamped UpdateVehicleTransform(const rclcpp::Time& current_time,
                                                                      const interface::VehicleState& vehicle_state,
                                                                      const std::string& vehicle_namespace) {
        tf2::Quaternion q;    
        q.setRPY(0, 0, vehicle_state.yaw);
        q.normalize();

        tf2::Transform vehicle_transform;
        vehicle_transform.setOrigin(tf2::Vector3(vehicle_state.x, vehicle_state.y, 0.0));
        vehicle_transform.setRotation(q);

        // broadcasting the vehicle's body coordinate system
        // The parent coordinate is world, child coordinate is body.
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = "world";
        msg.child_frame_id = vehicle_namespace + "/body";
        msg.transform.translation.x = vehicle_transform.getOrigin().x();
        msg.transform.translation.y = vehicle_transform.getOrigin().y();
        msg.transform.translation.z = vehicle_transform.getOrigin().z();
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        return msg;
    }

    inline ad_msgs::msg::VehicleCommand UpdateVehicleCommand(const interface::VehicleCommand& vehicle_command) {
        ad_msgs::msg::VehicleCommand msg;
        msg.accel = vehicle_command.accel;
        msg.brake = vehicle_command.brake;
        msg.steering = vehicle_command.steering;
        
        return msg;
    }
}

#endif // __ROS2_BRIDGE_VEHICLE_HPP__