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

struct ObstacleInfo {
    double x_ego;
    double y_ego;
    double velocity;
};

class AutonomousDriving : public rclcpp::Node {
    public:
        explicit AutonomousDriving(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~AutonomousDriving();

        void ProcessParams();
        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions     
        void DetectLaneShift(double ego_x, double ego_y, double current_time_seconds, double lane_width, int &current_lane);
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
        std::vector<double> generateSavitzkyGolayKernel(int window_size, int poly_order);
        std::vector<double> applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel);
        std::vector<std::vector<geometry_msgs::msg::Point>> dbscanClustering(
            const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight);
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
        double param_pp_kd_ = 1.0;
        double param_pp_kv_ = 0.05;
        double param_pp_kc_ = 0.5;
        double param_pid_kp_ = 5.0;
        double param_pid_ki_ = 0.002;
        double param_pid_kd_ = 0.01;
        // double param_brake_ratio_ = 1.2;
        double param_brake_ratio_ = 2.0;

        double param_m_ROIFront_param = 20.0;
        double param_m_ROIRear_param = 10.0;
        double param_m_ROILeft_param = 3.0;
        double param_m_ROIRight_param = 3.0;
        std::string param_ref_csv_path;

        // Custom variables //
        // Lane detection & fitting //
        // DBSCAN algorithm
        double eps = 5.0;      // Maximum distance for a point to be considered part of a cluster
        double x_weight = 0.05; // Weight for x-dimension
        int min_points = 3;    // Minimum number of points to form a cluster

        // Savitzky-Golay filter
        int window_size = 7;  // window size: Odd number
        int poly_order = 2;   // Cubic smoothing poly order

        const double lane_threshold  = 1.5;          // lane_threshold : Threshold for classifying points as left or right lane
        const double lane_width      = 4.0;          // lane_width : the width of lane
        bool b_is_left_lane_empty = true;           // b_is_left_lane_empty : left lane is empty
        bool b_is_right_lane_empty = true;          // b_is_right_lane_empty : right lane is empty

        // Control Trigger
        bool b_trigger_merge = false;               // b_trigger_merge : trigger for lane merge
        bool b_is_icy_road = false;
        bool b_is_up_slope = false;
        bool b_is_down_slope = false;
        bool b_left_merge = false;
        bool b_right_merge = false;
        bool b_is_merge_safe = true;
        bool b_lane_shifted = false;
        int current_lane             = 0;           // current driving lane ID; left=-1; middle=0; right=1
        bool b_return_to_center = false;            // trigger for returning to center lane
        bool b_vehicle_behind = false;              // trigger for vehicle behind
        double last_lane_shift_time_ = 0.0;               // last_lane_shift_time : time of the last lane shift

        // Path Planning
        double lane_shift_threshold = 3.0;   // Threshold for lane shift
        double shift_distance = 2.2;         // shift_distance : distance to shift the lane
        double prev_lane_center = 0.0;             // prev_lane_center : previous lane center
        double target_lane_center = 0.0;           // target_lane_center : target lane center
        double obs_look_ahead_dist = 8.0;     // obs_look_ahead_dist : look-ahead distance for obstacle avoidance
        double flank_dist_x = 2.0;            // flank_dist : x-distance from the vehicle to consider within merge
        double flank_dist_y = 2.0;            // flank_dist : y-distance from the vehicle to consider within merge
        double cutting_dist = 16.0;           // cutting_dist : distance for cutting in front of the vehicle
        double merge_start_x = 0.0;           // merge_start_x : x-coordinate to start merging
        double merge_start_y = 0.0;           // merge_start_y : y-coordinate to start merging
        double merge_start_yaw = 0.0;         // merge_start_yaw : yaw angle to start merging
        double behind_vehicle_speed = 0.0;    // behind_vehicle_speed : speed of the vehicle behind
        double emergency_braking_dist = 16.0; // emergency_braking_dist : distance for emergency braking in obstacle avoidance

        // Longitudinal Control
        double target_speed          = 10.0;
        double icy_speed       = 12.0;         // speed on icy road
        double merge_speed     = 8.0;         // speed for lane merge
        double braking_speed   = 3.5;         // speed for emergency braking
        double speed_error           = 0.0;          // PID error
        double speed_error_integral_ = 0.0;
        double speed_error_prev_     = 0.0;
        const double integral_max    = 4.0;          // for anti-windup
        const double curve_speed       = 5.0;          // minimum speed if steering exceeds steering_threshold
        double min_speed             = 0.0;          // minimum speed
        const double interval        = 0.01;         // time interval in seconds (100Hz=0.01s)
        const double slope_compensation_factor = 0.1; // up_slope_compensation_factor : compensation factor for uphill

        // Lateral Control
        double param_m_Lookahead_distance = 0.8;     // look-ahead dist for pure pursuit
        double lateral_error             = 0.0;
        double last_lateral_error        = 0.0;
        double max_steering_angle  = 0.35;
        double merge_steering_limit = 0.43; // steering limit for lane merge
        double alpha           = 0.5;
        const double steering_threshold = 0.18;      // steering threshold for triggering deceleration
        const double pursuit_threshold   = 12.0;    // for obstacle scenario
        const double safe_distance       = 11.0;        
};

#endif // __AUTONOMOUS_DRIVING_HPP__