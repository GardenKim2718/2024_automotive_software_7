/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      ros2_bridge_lane.hpp
 * @brief     ROS2 bridge for lane
 * 
 * @date      2024-11-06 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 *  
 */

#ifndef __ROS2_BRIDGE_LANE_HPP__
#define __ROS2_BRIDGE_LANE_HPP__
#pragma once

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Interface Header
#include "interface_lane.hpp"

namespace ros2_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline interface::Lanes GetLanePointsArray(const ad_msgs::msg::LanePointDataArray& msg) {
        interface::Lanes lane_points_array;
        lane_points_array.frame_id = msg.frame_id;
        lane_points_array.id = msg.id;

        for (const auto& lane : msg.lane) {
            interface::Lane lane_points;
            lane_points.frame_id = lane.frame_id;
            lane_points.id = lane.id;

            for (const auto& point : lane.point) {
                interface::Point2D lanePoint;
                lanePoint.x = point.x;
                lanePoint.y = point.y;
                lane_points.point.push_back(lanePoint);
            }
            lane_points_array.lane.push_back(lane_points);
        }

        return lane_points_array;
    }
    inline interface::Lane GetLanePoints(const ad_msgs::msg::LanePointData& msg) {
        interface::Lane lane_points;
        lane_points.frame_id = msg.frame_id;
        lane_points.id = msg.id;

        for (const auto& point : msg.point) {
            interface::Point2D lanePoint;
            lanePoint.x = point.x;
            lanePoint.y = point.y;
            lane_points.point.push_back(lanePoint);
        }

        return lane_points;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline ad_msgs::msg::LanePointDataArray UpdateCsvLanes(const interface::Lanes& csv_lanes) {
        ad_msgs::msg::LanePointDataArray msg;
        msg.frame_id = csv_lanes.frame_id;
        msg.id = csv_lanes.id;
        msg.lane.clear();

        for (const auto& csv_lane : csv_lanes.lane) {
            ad_msgs::msg::LanePointData lane;
            lane.frame_id = csv_lane.frame_id;
            lane.id = csv_lane.id;

            for (const auto& csv_point : csv_lane.point) {
                geometry_msgs::msg::PointStamped lanePoint_world;
                lanePoint_world.point.x = csv_point.x;
                lanePoint_world.point.y = csv_point.y;
                
                lane.point.push_back(lanePoint_world.point);
            }
            msg.lane.push_back(lane);
        }

        return msg;
    }

    inline ad_msgs::msg::LanePointDataArray UpdateROILanes(const interface::Lanes& roi_lanes) {
        ad_msgs::msg::LanePointDataArray msg;
        msg.frame_id = roi_lanes.frame_id;
        msg.id = roi_lanes.id;
        msg.lane.clear();

        for (const auto& roi_lane : roi_lanes.lane) {
            ad_msgs::msg::LanePointData lane;
            lane.frame_id = roi_lane.frame_id;
            lane.id = roi_lane.id;

            for (const auto& roi_point : roi_lane.point) {
                geometry_msgs::msg::PointStamped lanePoint_world;
                lanePoint_world.point.x = roi_point.x;
                lanePoint_world.point.y = roi_point.y;
                
                lane.point.push_back(lanePoint_world.point);
            }
            msg.lane.push_back(lane);
        }

        return msg;
    }

    inline ad_msgs::msg::LanePointData UpdateLanePoints(const interface::Lane& roi_lane,
                                                        const std::string vehicle_namespace) {
        ad_msgs::msg::LanePointData msg;
        msg.frame_id = vehicle_namespace + "/body";
        msg.id = roi_lane.id;

        for (const auto& point : roi_lane.point) {
            geometry_msgs::msg::Point lanepoints;
            lanepoints.x = point.x;
            lanepoints.y = point.y;
            msg.point.push_back(lanepoints);
        }

        return msg;
    }

    inline ad_msgs::msg::LanePointDataArray UpdateLanePointsArray(const interface::Lanes& roi_lanes,
                                                                  const std::string vehicle_namespace) {
        ad_msgs::msg::LanePointDataArray msg;
        msg.frame_id = vehicle_namespace + "/body";
        msg.id = roi_lanes.id;

        for (const auto& roi_lane : roi_lanes.lane) {
            ad_msgs::msg::LanePointData lane_points;
            lane_points.frame_id = vehicle_namespace + "/body";
            lane_points.id = roi_lane.id;

            for (const auto& point : roi_lane.point) {
                geometry_msgs::msg::Point lanepoints;
                lanepoints.x = point.x;
                lanepoints.y = point.y;
                lane_points.point.push_back(lanepoints);
            }
            msg.lane.push_back(lane_points);
        }

        return msg;
    }

    inline ad_msgs::msg::PolyfitLaneData UpdatePolyfitLane(const interface::PolyfitLane& lane) {
        ad_msgs::msg::PolyfitLaneData msg;
        msg.frame_id = lane.frame_id;
        msg.id = lane.id;

        msg.a0 = lane.a0;
        msg.a1 = lane.a1;
        msg.a2 = lane.a2;
        msg.a3 = lane.a3;

        return msg;
    }

    inline ad_msgs::msg::PolyfitLaneDataArray UpdatePolyfitLanes(const interface::PolyfitLanes& lanes) {
        ad_msgs::msg::PolyfitLaneDataArray msg;
        msg.frame_id = lanes.frame_id;
        msg.polyfitlanes.clear();

        for (const auto& lane : lanes.polyfitlanes) {
            ad_msgs::msg::PolyfitLaneData polyfit_lane;
            polyfit_lane.frame_id = lane.frame_id;
            polyfit_lane.id = lane.id;

            polyfit_lane.a0 = lane.a0;
            polyfit_lane.a1 = lane.a1;
            polyfit_lane.a2 = lane.a2;
            polyfit_lane.a3 = lane.a3;

            msg.polyfitlanes.push_back(polyfit_lane);
        }

        return msg;
    }
}

#endif // __ROS2_BRIDGE_LANE_HPP__