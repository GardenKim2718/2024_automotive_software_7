/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_algorithm.cpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __EVALUATION_ALGORITHM_HPP__
#define __EVALUATION_ALGORITHM_HPP__
#pragma once

// STD Header
#include <string>
#include <math.h>

// Interface Header
#include "interface_lane.hpp"
#include "interface_vehicle.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "evaluation_config.hpp"

using namespace interface;

class EvaluationAlgorithm {
    public:
        explicit EvaluationAlgorithm(const EvaluationConfig& params);
        virtual ~EvaluationAlgorithm();
        
    public:
        bool RunAlgorithm(const VehicleState& vehicle_state,
                          const Mission& mission,
                          const double& time_eval_driving,
                          const double& time_dt,
                          const double& gain,
                          const EvaluationConfig& cfg);

        inline std::string GetEvaluationInfo() { return o_evaluation_info_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        interface::Lane LoadLaneData(const EvaluationConfig& cfg);

        void CalcOverSpeedPenalty(const VehicleState& vehicle_state, const double& dt, const double& limit_speed);
        void CalcCrossTrackError(const VehicleState& vehicle_state, const double& dt);
        bool CheckCollision(const VehicleState& vehicle_state, const std::vector<MissionObject>& obstacles);
        void CheckFailure(const bool& is_collision, const double& driving_time, const EvaluationConfig& cfg);
        bool IsFinished(const VehicleState& vehicle_state, const double& gain);
        std::string UpdateEvaluationInfo(const VehicleState& vehicle_state, 
                                         const double& driving_time,
                                         const double& limit_speed,
                                         const EvaluationConfig& cfg);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Inputs        
        interface::Lane     i_ref_lane_;
        interface::Point2D  i_goal_point_;

        // Outputs
        std::string o_evaluation_info_;
        
        // Evaluation results
        double eval_speed_penalty_ = 0.0;

        double eval_curr_cte_ = 0.0;
        double eval_LAD_cte_ = 0.0;
        double eval_max_cte_ = 0.0;

        double eval_curr_spacing_error_ = 0.0;
        double eval_LAD_spacing_error_ = 0.0;

        bool eval_is_lane_departure_ = false;
        bool eval_is_collision_ = false;
        bool eval_is_retire_ = false;
};


#endif // __EVALUATION_ALGORITHM_HPP__
