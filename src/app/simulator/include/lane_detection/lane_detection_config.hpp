/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_config.hpp
 * @brief     lane detection configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __LANE_DETECTION_CONFIG_HPP__
#define __LANE_DETECTION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};
    std::string ref_csv_path{""};

    double loop_rate_hz{30.0};
    double ROI_front{20.0};
    double ROI_rear{10.0};
    double ROI_left{3.0};
    double ROI_right{3.0};

} LaneDetectionConfig;

#endif // __LANE_DETECTION_CONFIG_HPP__