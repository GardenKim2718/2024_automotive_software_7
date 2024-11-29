/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_config.hpp
 * @brief     evaluation configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __EVALUATION_CONFIG_HPP__
#define __EVALUATION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string ref_csv_path;
    std::string lane_id;

    double loop_rate_hz;
    double eval_time_limit;
    double eval_lane_departure;
} EvaluationConfig;

#endif // __EVALUATION_CONFIG_HPP__