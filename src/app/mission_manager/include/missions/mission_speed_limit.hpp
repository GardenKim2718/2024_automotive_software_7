/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      mission_speed_limit.hpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#ifndef __MISSION_SPEED_LIMIT_HPP__
#define __MISSION_SPEED_LIMIT_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "mission_manager_config.hpp"

class MissionSpeedLimit {
    public:
        explicit MissionSpeedLimit(const interface::RefLanes& ref_lanes,
                                   const int& id_center,
                                   const MissionManagerConfig& cfg);
        virtual ~MissionSpeedLimit();
    
    public:
        double RunAlgorithm(const interface::VehicleState& ego_vehicle_state,
                            const MissionManagerConfig& cfg);
        inline std::vector<interface::MissionRegion> GetSpeedLimit() { return container_speed_limit_zone_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        std::vector<interface::MissionRegion> GenerateSpeedLimitZone(const interface::RefLanes& ref_lanes,
                                                                     const int& id_center,
                                                                     const MissionManagerConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        std::vector<interface::MissionRegion> container_speed_limit_zone_;
};

#endif // __MISSION_SPEED_LIMIT_HPP__
