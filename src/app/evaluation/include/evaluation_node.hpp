/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_node.hpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2018-11-28 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __EVALUATION_NODE_HPP__
#define __EVALUATION_NODE_HPP__
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
#include "ros2_bridge_mission.hpp"
#include "ros2_bridge_evaluation.hpp"

// Algorithm Header
#include "evaluation_algorithm.hpp"

class Evaluation : public rclcpp::Node {
    public:
        explicit Evaluation(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~Evaluation();

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
        inline void CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_mission_);
            i_mission_ = ros2_bridge::GetMission(*msg);
            b_is_mission_ = true;
        }
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr      s_mission_;
        
        // Input
        interface::VehicleState i_vehicle_state_;
        interface::Mission      i_mission_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_mission_;
        
        // Publisher
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr p_text_evaluation_result_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Algorithm
        std::unique_ptr<EvaluationAlgorithm> alg_evaluation_;

        // Util and Configuration
        EvaluationConfig cfg_;

        // Flag
        bool b_is_simulator_on_ = false;
        bool b_is_mission_ = false;
        bool b_is_finished_ = false;

        // Global Variables   
        double time_start_ = 0.0;
        double time_prev_ = 0.0;
        double time_dt_ = 0.0;
        double time_eval_driving_ = 0.0;
}; 

#endif // __EVALUATION_NODE_HPP__
