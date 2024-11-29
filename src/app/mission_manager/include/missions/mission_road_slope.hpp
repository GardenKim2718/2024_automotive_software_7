/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      mission_road_slope.hpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __MISSION_ROAD_SLOPE_HPP__
#define __MISSION_ROAD_SLOPE_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "mission_manager_config.hpp"

class MissionRoadSlope {
    public:
        explicit MissionRoadSlope(const interface::RefLanes& ref_lanes,
                                  const int& id_center,
                                  const MissionManagerConfig& cfg);
        virtual ~MissionRoadSlope();
    
    public:
        std::string RunAlgorithm(const interface::VehicleState& ego_vehicle_state,
                                 const MissionManagerConfig& cfg);
        inline std::vector<interface::MissionRegion> GetRoadSlope() { return container_road_slope_; }
        
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        std::vector<interface::MissionRegion> GenerateRoadSlope(const interface::RefLanes& ref_lanes,
                                                                const int& id_center,
                                                                const MissionManagerConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        std::vector<interface::MissionRegion> container_road_slope_;
};

#endif // __MISSION_ROAD_SLOPE_HPP__
