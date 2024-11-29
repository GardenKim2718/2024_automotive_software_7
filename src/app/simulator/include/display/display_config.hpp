/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      display_config.hpp
 * @brief     display configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __DISPLAY_CONFIG_HPP__
#define __DISPLAY_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};
    std::string mesh_dir{""};

    double loop_rate_hz{100.0};
} DisplayConfig;

#endif // __DISPLAY_CONFIG_HPP__