/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_algorithm.hpp
 * @brief     vehicle simulator algorithm
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __VEHICLE_SIMULATOR_ALGORITHM_HPP__
#define __VEHICLE_SIMULATOR_ALGORITHM_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_mission.hpp"

// Parameter Header
#include "vehicle/vehicle_simulator_config.hpp"

using namespace interface;

class VehicleSimulatorAlgorithm {
    public:
        explicit VehicleSimulatorAlgorithm(const VehicleSimulatorConfig& cfg);
        virtual ~VehicleSimulatorAlgorithm();
    
    public:
        interface::VehicleState RunAlgorithm(const interface::VehicleCommand& vehicle_command,
                                             const interface::Mission& mission,
                                             const double& time_dt,
                                             const VehicleSimulatorConfig& cfg);
        inline interface::VehicleState GetInitialVehicleState() { return o_initial_vehicle_state_; }
                                             
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        interface::VehicleState SimVehicleLongitudinalModel(const interface::VehicleState& prev_vehicle_state,
                                                            const interface::VehicleCommand& vehicle_command,
                                                            const interface::Mission& mission,
                                                            const double& time_dt,
                                                            const VehicleSimulatorConfig& cfg);
        interface::VehicleState SimVehicleLateralModel(const interface::VehicleState& prev_vehicle_state,
                                                       const interface::VehicleCommand& vehicle_command,
                                                       const interface::Mission& mission,
                                                       const double& time_dt,
                                                       const VehicleSimulatorConfig& cfg);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        
        
        // Outputs
        interface::VehicleState o_initial_vehicle_state_;
        interface::VehicleState o_vehicle_state_;
};

#endif // __VEHICLE_SIMULATOR_ALGORITHM_HPP__