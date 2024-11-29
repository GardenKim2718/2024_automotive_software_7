/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_config.hpp
 * @brief     simulate vehicle configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __VEHICLE_SIMULATOR_CONFIG_HPP__
#define __VEHICLE_SIMULATOR_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};

    double loop_rate_hz{100.0};
    double init_x{0.0};
    double init_y{0.0};
    double init_pitch{0.0};
    double init_yaw{0.0};
    double init_vel{0.0};

    double uphill_slope{10.0};
    double downhill_slope{-10.0};
} VehicleSimulatorConfig;

#endif // __VEHICLE_SIMULATOR_CONFIG_HPP__