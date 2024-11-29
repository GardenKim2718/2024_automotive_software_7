/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      autonomous_driving_config.hpp
 * @brief     autonomous driving configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __AUTONOMOUS_DRIVING_CONFIG_HPP__
#define __AUTONOMOUS_DRIVING_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};
    double loop_rate_hz{100.0};
    bool use_manual_inputs{false};

    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////
} AutonomousDrivingConfig;

#endif // __AUTONOMOUS_DRIVING_CONFIG_HPP__