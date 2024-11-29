/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      interface_scenario.hpp
 * @brief     scenario component structure
 * 
 * @date      2024-10-21 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __INTERFACE_MISSION_HPP__
#define __INTERFACE_MISSION_HPP__
#pragma once

#include <vector>
#include <string>

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        double x{0.0};
        double y{0.0};
        double s{0.0};
    } RefLanePoint;
    
    typedef struct {
        std::vector<RefLanePoint> points;
    } RefLane;

    typedef struct {
        std::vector<RefLane> lanes;
    } RefLanes;

    typedef struct{
        double          x{0.0};
        double          y{0.0};
        double          radius{0.0};
        std::string     mission{"None"};
        std::string     sub_type{"None"};   // Uphill, Downhill, Ice
    } MissionRegion;

    typedef struct {
        std::string object_type{"None"};    // Dynamic, Static
        bool is_reach_end{false};
        int ref_lane_id{0};
        double x{0.0};
        double y{0.0};
        double s{0.0};                      // [m]
        double yaw{0.0};                    // [rad]
        double velocity{0.0};               // [mps]
        double start{0.0};                  // s[m]
        double end{0.0};                    // s[m]
        double target_velocity{0.0};        // [mps]
    } MissionObject;

    typedef struct {
        std::vector<MissionObject>  objects;
        std::string                 road_condition{"Asphalt"};
        std::string                 road_slope{"None"};
        double                      speed_limit{120.0/3.6};
    } Mission;

} // namespace interface

#endif // __INTERFACE_MISSION_HPP__