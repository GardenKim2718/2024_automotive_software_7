/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 * 
 * @file      autonomous_driving_node.hpp
 * @brief     autonomous driving node
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __AUTONOMOUS_DRIVING_NODE_HPP__
#define __AUTONOMOUS_DRIVING_NODE_HPP__
#pragma once

// STD Header
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
#include "ros2_bridge_mission.hpp"

// Parameter Header
#include "autonomous_driving_config.hpp"

class AutonomousDriving : public rclcpp::Node {
    public:
        explicit AutonomousDriving(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~AutonomousDriving();

        void ProcessParams();
        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions     
        
        // Callback functions   
        inline void CallbackManualInput(const ad_msgs::msg::VehicleCommand::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_manual_input_);
            if (cfg_.use_manual_inputs == true) {
                i_manual_input_ = ros2_bridge::GetVehicleCommand(*msg);
                b_is_manual_input_ = true;
            }
        }
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {            
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros2_bridge::GetVehicleState(*msg);
            b_is_simulator_on_ = true;
        }
        inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {            
            std::lock_guard<std::mutex> lock(mutex_lane_points_);
            i_lane_points_ = ros2_bridge::GetLanePoints(*msg);
            b_is_lane_points_ = true;
        }
        inline void CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_mission_);
            i_mission_ = ros2_bridge::GetMission(*msg);
            b_is_mission_ = true;
        }

        ////////////////////// TODO //////////////////////
        // TODO: Add more functions

        //////////////////////////////////////////////////

        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleCommand>::SharedPtr       s_manual_input_;
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr         s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr        s_lane_points_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr              s_mission_;
        
        // Input
        interface::VehicleCommand   i_manual_input_;
        interface::VehicleState     i_vehicle_state_;
        interface::Lane             i_lane_points_;
        interface::Mission          i_mission_;

        // Mutex
        std::mutex mutex_manual_input_;
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_lane_points_;
        std::mutex mutex_mission_;

        // Publisher
        rclcpp::Publisher<ad_msgs::msg::VehicleCommand>::SharedPtr          p_vehicle_command_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr         p_driving_way_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr    p_poly_lanes_;
        
        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Util and Configuration
        AutonomousDrivingConfig cfg_;
                
        // Flag
        bool b_is_manual_input_ = false;
        bool b_is_simulator_on_ = false;
        bool b_is_lane_points_ = false;
        bool b_is_mission_ = false;
};

#endif // __AUTONOMOUS_DRIVING_HPP__