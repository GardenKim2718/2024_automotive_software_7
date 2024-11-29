/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_node.hpp
 * @brief     simulate lane detection
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __LANE_DETECTION_NODE_HPP__
#define __LANE_DETECTION_NODE_HPP__
#pragma once

// STD Header
#include <cstdlib>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

// Bridge Header
#include "ros2_bridge_vehicle.hpp"
#include "ros2_bridge_lane.hpp"

// Algorithm Header
#include "lane_detection/lane_detection_algorithm.hpp"

class LaneDetection : public rclcpp::Node {
    public:
        explicit LaneDetection(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~LaneDetection();

        void ProcessParams();
        void Run();
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Function

        // Callback function
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros2_bridge::GetVehicleState(*msg);
            b_is_simulator_on_ = true;
        }
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable
        
        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;

        // Input
        interface::VehicleState i_vehicle_state_;

        // Mutex
        std::mutex mutex_vehicle_state_;
                
        // Publisher
        rclcpp::Publisher<ad_msgs::msg::LanePointDataArray>::SharedPtr p_csv_lanes_;
        rclcpp::Publisher<ad_msgs::msg::LanePointDataArray>::SharedPtr p_roi_lanes_;
        rclcpp::Publisher<ad_msgs::msg::LanePointData>::SharedPtr p_lane_points_;
        
        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Algorithm
        std::unique_ptr<LaneDetectionAlgorithm> alg_lane_detection_;
        
        // Util and Configuration
        LaneDetectionConfig cfg_;
        
        // Flag
        bool b_is_simulator_on_ = false;
        
        // Global Variable
        double time_prev_csv_lanes_ = 0.0;
};

#endif // __LANE_DETECTION_NODE_HPP__