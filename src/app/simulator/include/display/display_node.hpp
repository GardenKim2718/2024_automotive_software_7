/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      display_node.hpp
 * @brief     display topics using markers
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-10-12 updated by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *              : added dynamic obstacle vehicle
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __DISPLAY_NODE_HPP__
#define __DISPLAY_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// ROS Message Header
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/mission.hpp>
#include <ad_msgs/msg/mission_object.hpp>
#include <ad_msgs/msg/mission_region.hpp>
#include <ad_msgs/msg/mission_display.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>

// Parameter Header
#include "display/display_config.hpp"

class Display : public rclcpp::Node {
    public:
        explicit Display(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~Display();

        void ProcessParams();
        void Run();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Function

        // Callback function
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            i_vehicle_state_ = *msg;
            b_is_vehicle_state_ = true;
        }
        inline void CallbackMission(const ad_msgs::msg::MissionDisplay::SharedPtr msg) {
            i_mission_ = *msg;
            b_is_mission_ = true;
        }
        inline void CallbackCsvLanes(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {            
            i_csv_lanes_ = *msg;
            b_is_csv_lanes_ = true;
        }
        inline void CallbackROILanes(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {
            i_roi_lanes_ = *msg;
            b_is_roi_lanes_ = true;
        }
        inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
            i_lane_points_ = *msg;
            b_is_lane_points_ = true;
        }
        inline void CallbackPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg) {
            i_poly_lanes_ = *msg;
            b_is_poly_lanes_ = true;
        }
        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            i_driving_way_ = *msg;
            b_is_driving_way_ = true;
        }

        // Algorithm function
        void DisplayVehicle(const ad_msgs::msg::VehicleState& vehicle_state,
                            const rclcpp::Time& current_time,
                            const DisplayConfig& cfg);
        void DisplayMission(const ad_msgs::msg::MissionDisplay& mission,
                            const ad_msgs::msg::VehicleState& vehicle_state,
                            const rclcpp::Time& current_time,
                            const DisplayConfig& cfg);
        void DisplayROILanes(const ad_msgs::msg::LanePointDataArray& roi_lanes,
                             const rclcpp::Time& current_time);
        void DisplayLanePoints(const ad_msgs::msg::LanePointData& lane_points,
                               const rclcpp::Time& current_time);
        void DisplayPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray& poly_lanes,
                              const rclcpp::Time& current_time,
                              const double& interval, const double& ROILength);
        void DisplayDrivingWay(const ad_msgs::msg::PolyfitLaneData& driving_way,
                               const rclcpp::Time& current_time,
                               const double& interval, const double& ROILength);
        void DisplayCsvLanes(const ad_msgs::msg::LanePointDataArray& csv_lanes,
                             const rclcpp::Time& current_time);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variable

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr         s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::MissionDisplay>::SharedPtr       s_mission_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr   s_csv_lanes_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr   s_roi_lanes_;
        rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr        s_lane_points_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr s_poly_lanes_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr      s_driving_way_;

        // Input
        ad_msgs::msg::VehicleState          i_vehicle_state_;
        ad_msgs::msg::MissionDisplay        i_mission_;
        ad_msgs::msg::LanePointDataArray    i_csv_lanes_;
        ad_msgs::msg::LanePointData         i_lane_points_;
        ad_msgs::msg::LanePointDataArray    i_roi_lanes_;
        ad_msgs::msg::PolyfitLaneDataArray  i_poly_lanes_;
        ad_msgs::msg::PolyfitLaneData       i_driving_way_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_mission_;
        std::mutex mutex_csv_lanes_;
        std::mutex mutex_lane_points_;
        std::mutex mutex_roi_lanes_;
        std::mutex mutex_poly_lanes_;
        std::mutex mutex_driving_way_;
        
        // Publisher
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       p_vehicle_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  p_mission_marker_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                p_ego_vehicle_velocity_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  p_csv_lanes_marker_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       p_lane_points_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  p_roi_lanes_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  p_poly_lanes_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  p_driving_way_marker_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Util and Configuration
        DisplayConfig cfg_;

        // Flag    
        bool b_is_vehicle_state_    = false; 
        bool b_is_mission_          = false;
        bool b_is_csv_lanes_        = false;
        bool b_is_roi_lanes_        = false;
        bool b_is_lane_points_      = false;
        bool b_is_poly_lanes_       = false;
        bool b_is_driving_way_      = false;

        // Global Variable
        double time_vehicle_marker_ = 0.0;
        double time_csv_lanes_marker_ = 0.0;
        double time_roi_lanes_marker_ = 0.0;
        double time_lane_points_marker_ = 0.0;
        double time_poly_lanes_marker_ = 0.0;
        double time_driving_way_marker_ = 0.0;
};

#endif // __DISPLAY_NODE_HPP__
