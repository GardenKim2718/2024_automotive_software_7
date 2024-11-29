/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      ros2_bridge_mission.hpp
 * @brief     ROS2 bridge for mission
 * 
 * @date      2024-11-06 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 *  
 */

#ifndef __ROS2_BRIDGE_MISSION_HPP__
#define __ROS2_BRIDGE_MISSION_HPP__
#pragma once

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header
#include <ad_msgs/msg/mission.hpp>
#include <ad_msgs/msg/mission_object.hpp>
#include <ad_msgs/msg/mission_region.hpp>
#include <ad_msgs/msg/mission_display.hpp>

// Interface Header
#include "interface_mission.hpp"

namespace ros2_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline interface::Mission GetMission(const ad_msgs::msg::Mission& msg) {
        interface::Mission mission;
        mission.speed_limit = msg.speed_limit;
        mission.road_condition = msg.road_condition;
        mission.road_slope = msg.road_slope;

        mission.objects.clear();
        for (const auto& obj : msg.objects) {
            interface::MissionObject object;
            object.object_type  = obj.object_type;
            object.is_reach_end = obj.is_reach_end;
            object.x            = obj.x;
            object.y            = obj.y;
            object.yaw          = obj.yaw;
            object.velocity     = obj.velocity;

            mission.objects.push_back(object);
        }

        return mission;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    inline ad_msgs::msg::Mission UpdateMission(const std::vector<interface::MissionObject>& objects_scc,
                                               const std::vector<interface::MissionObject>& objects_merge,
                                               const std::string& road_condition,
                                               const std::string& road_slope,
                                               const double& speed_limit) {
        ad_msgs::msg::Mission msg;
        msg.speed_limit     = speed_limit;
        msg.road_condition  = road_condition;
        msg.road_slope      = road_slope;
        
        msg.objects.clear();
        for(const auto& obj : objects_scc) {
            ad_msgs::msg::MissionObject ob;
            if(ob.is_reach_end == false) {
                ob.object_type  = obj.object_type;
                ob.is_reach_end = obj.is_reach_end;
                ob.x            = obj.x;
                ob.y            = obj.y;
                ob.yaw          = obj.yaw;
                ob.velocity     = obj.velocity;

                msg.objects.push_back(ob);
            }
        }

        for(const auto& obj : objects_merge) {
            ad_msgs::msg::MissionObject ob;
            ob.object_type  = obj.object_type;
            ob.is_reach_end = obj.is_reach_end;
            ob.x            = obj.x;
            ob.y            = obj.y;
            ob.yaw          = obj.yaw;
            ob.velocity     = obj.velocity;

            msg.objects.push_back(ob);
        }

        return msg;
    }

    inline ad_msgs::msg::MissionDisplay UpdateMissionDisplay(const std::vector<interface::MissionObject>& display_scc,
                                                             const std::vector<interface::MissionObject>& display_merge,
                                                             const std::vector<interface::MissionRegion>& display_road_slope,
                                                             const std::vector<interface::MissionRegion>& display_road_condition,
                                                             const std::vector<interface::MissionRegion>& display_speed_limit) {
        ad_msgs::msg::MissionDisplay msg;
        
        for(auto scc : display_scc) {
            ad_msgs::msg::MissionObject obj;
            if(scc.is_reach_end == false) {
                obj.object_type     = scc.object_type;
                obj.x               = scc.x;
                obj.y               = scc.y;
                obj.yaw             = scc.yaw;
                obj.velocity        = scc.velocity;
                obj.is_reach_end    = scc.is_reach_end;

                msg.objects.push_back(obj);
            }
        }

        for(auto merge : display_merge) {
            ad_msgs::msg::MissionObject obj;
            obj.object_type     = merge.object_type;
            obj.is_reach_end    = merge.is_reach_end;
            obj.x               = merge.x;
            obj.y               = merge.y;
            obj.yaw             = merge.yaw;
            obj.velocity        = merge.velocity;

            msg.objects.push_back(obj);
        }

        for(auto mission : display_road_slope) {
            ad_msgs::msg::MissionRegion region;
            region.x            = mission.x;
            region.y            = mission.y;
            region.radius       = mission.radius;
            region.mission      = mission.mission;
            region.sub_type     = mission.sub_type;

            msg.regions.push_back(region);
        }

        for(auto mission : display_road_condition) {
            ad_msgs::msg::MissionRegion region;
            region.x            = mission.x;
            region.y            = mission.y;
            region.radius       = mission.radius;
            region.mission      = mission.mission;
            region.sub_type     = mission.sub_type;

            msg.regions.push_back(region);
        }

        for(auto mission : display_speed_limit) {
            ad_msgs::msg::MissionRegion region;
            region.x            = mission.x;
            region.y            = mission.y;
            region.radius       = mission.radius;
            region.mission      = mission.mission;
            region.sub_type     = mission.sub_type;

            msg.regions.push_back(region);
        }

        return msg;
    }
}

#endif // __ROS2_BRIDGE_MISSION_HPP__