/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_node.hpp
 * @brief     simulate vehicle
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __VEHICLE_SIMULATOR_NODE_HPP__
#define __VEHICLE_SIMULATOR_NODE_HPP__
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
#include "ros2_bridge_mission.hpp"

// Algorithm Header
#include "vehicle/vehicle_simulator_algorithm.hpp"

class Vehicle : public rclcpp::Node {
    public:
        explicit Vehicle(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~Vehicle();

        void ProcessParams();
        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Function

        // Callback function
        inline void CallbackVehicleCommand(const ad_msgs::msg::VehicleCommand::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_command_);
            i_vehicle_command_ = ros2_bridge::GetVehicleCommand(*msg);
            b_is_vehicle_command_ = true;
        }
        inline void CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_mission_);
            i_mission_ = ros2_bridge::GetMission(*msg);
            b_is_mission_ = true;
        }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleCommand>::SharedPtr s_vehicle_command_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr        s_mission_;

        // Input
        interface::VehicleCommand i_vehicle_command_;
        interface::Mission        i_mission_;

        // Mutex
        std::mutex mutex_vehicle_command_;
        std::mutex mutex_mission_;

        // Publisher
        rclcpp::Publisher<ad_msgs::msg::VehicleState>::SharedPtr p_vehicle_state_;
        
        // Tf
        tf2_ros::TransformBroadcaster tf2_broadcaster_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // Algorithm
        std::unique_ptr<VehicleSimulatorAlgorithm> alg_vehicle_simulator_;

        // Util and Configuration
        VehicleSimulatorConfig cfg_;
        
        // Flag
        bool b_is_vehicle_command_ = false;
        bool b_is_mission_ = false;
        
        // Global Variable    
        double time_prev_ = 0.0;
};

#endif // __VEHICLE_SIMULATOR_NODE_HPP__
