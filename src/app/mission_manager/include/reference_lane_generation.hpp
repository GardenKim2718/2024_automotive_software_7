/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      reference_lane_generation.hpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-11-06 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 * 
 */


#ifndef __REFERENCE_LANE_GENERATION_HPP__
#define __REFERENCE_LANE_GENERATION_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_lane.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "mission_manager_config.hpp"

using namespace interface;

class ReferenceLaneGeneration {
    public:
        ReferenceLaneGeneration(const MissionManagerConfig& cfg);
        ~ReferenceLaneGeneration();
        
    public:
        inline interface::RefLanes GetReferenceLanes() { return o_ref_lanes_; }
        inline int GetCenterLaneId() { return o_id_center_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        interface::RefLanes LoadLanesData(const MissionManagerConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        interface::RefLanes o_ref_lanes_;
        int o_id_center_ = 0;
};

#endif // __REFERENCE_LANE_GENERATION_HPP__
