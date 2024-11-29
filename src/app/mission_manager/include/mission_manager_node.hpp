/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      scenario_runner_node.hpp
 * @brief     generate mission events node
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __MISSION_MANAGER_NODE_HPP__
#define __MISSION_MANAGER_NODE_HPP__
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

// Algorithm Header
#include "reference_lane_generation.hpp"
#include "missions/mission_scc.hpp"
#include "missions/mission_road_slope.hpp"
#include "missions/mission_road_condition.hpp"
#include "missions/mission_speed_limit.hpp"
#include "missions/mission_merge.hpp"

class ScenarioRunner : public rclcpp::Node {
    public:
        explicit ScenarioRunner(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ScenarioRunner();

        void ProcessParams();
        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Function
        
        // Callback function
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros2_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
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
        rclcpp::Publisher<ad_msgs::msg::Mission>::SharedPtr         p_detected_mission_;
        rclcpp::Publisher<ad_msgs::msg::MissionDisplay>::SharedPtr  p_display_mission_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;        

        // Algorithm
        std::unique_ptr<ReferenceLaneGeneration>    alg_reference_lane_generation_;
        std::unique_ptr<MissionRoadCondition>       alg_mission_road_condition_;
        std::unique_ptr<MissionRoadSlope>                alg_mission_road_slope_;
        std::unique_ptr<MissionSCC>                 alg_mission_scc_;
        std::unique_ptr<MissionSpeedLimit>          alg_mission_speed_limit_;
        std::unique_ptr<MissionMerge>               alg_mission_merge_;

        // Util and Configuration
        MissionManagerConfig cfg_;

        // Flags
        bool b_is_vehicle_state_ = false;

        // Global Variable
        double time_prev_ = 0.0;
        interface::RefLanes i_ref_lanes_;
        int i_id_center_ = 0;
};

#endif // __MISSION_MANAGER_NODE_HPP__
