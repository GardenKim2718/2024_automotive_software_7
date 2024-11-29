/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_algorithm.hpp
 * @brief     lane detection algorithm
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __LANE_DETECTION_ALGORITHM_HPP__
#define __LANE_DETECTION_ALGORITHM_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>
#include <algorithm>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_lane.hpp"

// Parameter Header
#include "lane_detection/lane_detection_config.hpp"

using namespace interface;

class LaneDetectionAlgorithm {
    public:
        explicit LaneDetectionAlgorithm(const LaneDetectionConfig& cfg);
        virtual ~LaneDetectionAlgorithm();
    
    public:
        interface::Lanes RunAlgorithm(const VehicleState& vehicle_state,
                                      const LaneDetectionConfig& cfg);
        inline interface::Lanes GetRefLanes() { return o_ref_lanes_; }
        inline interface::Lane GetRefRandomLane() { return o_ref_random_lane_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        interface::Lanes LoadLanesData(const LaneDetectionConfig& cfg);
        interface::Point2D TfFromWorldToBody(const interface::Point2D& point_world, 
                                             const VehicleState& vehicle_state);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Outputs
        interface::Lanes o_ref_lanes_;
        interface::Lane o_ref_random_lane_;
};

#endif // __LANE_DETECTION_ALGORITHM_HPP__