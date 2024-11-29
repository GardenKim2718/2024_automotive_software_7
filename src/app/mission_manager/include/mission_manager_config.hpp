/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      mission_manager_config.hpp
 * @brief     mission manager configuration
 * 
 * @date      2024-10-18 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __MISSION_MANAGER_CONFIG_HPP__
#define __MISSION_MANAGER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};
    double loop_rate_hz{100.0};

    std::string ref_csv_path;
    int num_of_lane{3};
    int num_of_merging_vehicles{3};
    int id_merging_lane{0};
    double detection_range{20.0};
    double object_max_accel{3.0};
    double max_velocity{120.0};
    double merging_section_length{25.0};
    double merging_interval{10.0};
    double merging_speed{30.0};
    double merging_target_speed{50.0};
    double merging_section_length_margin{2.0};
} MissionManagerConfig;

#endif // __MISSION_MANAGER_CONFIG_HPP__