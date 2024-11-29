/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      ros2_bridge_evaluation.hpp
 * @brief     ROS2 bridge for evaluation
 * 
 * @date      2024-11-06 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 *  
 */

#ifndef __ROS2_BRIDGE_EVALUATION_HPP__
#define __ROS2_BRIDGE_EVALUATION_HPP__
#pragma once

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>

// Interface Header
#include "interface_mission.hpp"

namespace ros2_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline rviz_2d_overlay_msgs::msg::OverlayText UpdateEvaluationResult(const std::string& evaluation_info) {
        rviz_2d_overlay_msgs::msg::OverlayText msg;
        msg.bg_color.r = 0.0f;
        msg.bg_color.g = 0.0f;
        msg.bg_color.b = 0.0f;
        msg.bg_color.a = 0.0f;

        msg.fg_color.r = 0.9f;
        msg.fg_color.g = 0.9f;
        msg.fg_color.b = 0.9f;
        msg.fg_color.a = 0.7f;
        
        msg.line_width = 1;
        msg.text_size = 9.0;
        msg.text = evaluation_info;

        return msg;
    }
}

#endif // __ROS2_BRIDGE_EVALUATION_HPP__