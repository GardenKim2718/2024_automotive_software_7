/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      mission_merge.hpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */


#ifndef __MISSION_MERGE_HPP__
#define __MISSION_MERGE_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "mission_manager_config.hpp"

class MissionMerge {
    public:
        explicit MissionMerge(const interface::RefLanes& ref_lanes,
                              const int& id_center,
                              const MissionManagerConfig& cfg);
        virtual ~MissionMerge();

    public:
        std::vector<interface::MissionObject> RunAlgorithm(const interface::VehicleState& ego_vehicle_state,
                                                           const interface::RefLanes& ref_lanes,
                                                           const double dt,
                                                           const MissionManagerConfig& cfg);
        inline std::vector<interface::MissionObject> GetMergeObjects() { return container_merge_objects_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        std::vector<interface::MissionObject> GenerateMergeObjects(const interface::RefLanes& ref_lanes,
                                                                   const int& id_center,
                                                                   const MissionManagerConfig& cfg);
        interface::MissionObject CalculateObjectLanePose(const interface::RefLanes& ref_lanes,
                                                         const interface::MissionObject obj);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        std::vector<interface::MissionObject> container_merge_objects_;
};

#endif // __MISSION_MERGE_HPP__
