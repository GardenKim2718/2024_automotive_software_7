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

// algorithm Header
#include <eigen3/Eigen/Dense>

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

        // Parameters
        // <arg name="pure_pursuit_kd" default="5.0"/>
        // <arg name="pure_pursuit_kv" default="0.35"/>
        // <arg name="pure_pursuit_kc" default="0.0"/>
        // <arg name="pid_kp" default="5.0"/>
        // <arg name="pid_ki" default="0.002"/>
        // <arg name="pid_kd" default="0.0"/>
        // <arg name="brake_ratio" default="1.2"/>
        double param_pp_kd_ = 1.0;
        double param_pp_kv_ = 0.05;
        double param_pp_kc_ = 0.5;
        double param_pid_kp_ = 5.0;
        double param_pid_ki_ = 0.002;
        double param_pid_kd_ = 0.01;
        double param_brake_ratio_ = 1.2;

        double param_m_ROIFront_param = 20.0;
        double param_m_ROIRear_param = 10.0;
        double param_m_ROILeft_param = 3.0;
        double param_m_ROIRight_param = 3.0;
        std::string param_ref_csv_path;

        // Algorhtm variables
        double speed_error_integral_ = 0.0;
        double speed_error_prev_     = 0.0;

        // Custom variables
        double target_speed          = 10.0;
        double speed_error           = 0.0;          // PID error
        // rclcpp::Time last_time;
        const double min_speed       = 3.0;          // minimum speed if steering exceeds steering_threshold
        const double alpha           = 0.5;
        const double steering_threshold = 0.18;      // steering threshold for triggering deceleration
        const double interval        = 0.01;         // time interval in seconds (100Hz=0.01s)
        const double integral_max    = 4.0;          // for anti-windup
        double param_m_Lookahead_distance = 0.8;     // look-ahead dist for pure pursuit
        const double pursuit_threshold     = 12.0;    // for obstacle scenario
        const double safe_distance         = 11.0;
        double lateral_error             = 0.0;
        double last_lateral_error        = 0.0;
        const double max_steering_angle = 0.35;
        
};

#endif // __AUTONOMOUS_DRIVING_HPP__