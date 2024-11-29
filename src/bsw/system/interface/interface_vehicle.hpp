/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      interface_vehicle_state.hpp
 * @brief     VehicleState structure
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __INTERFACE_VEHICLE_STATE_HPP__
#define __INTERFACE_VEHICLE_STATE_HPP__
#pragma once

#include <vector>
#include <string>

namespace interface {    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // constants
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  
    namespace vehicle_param {
        const double max_steer      = 30.0*M_PI/180.0;  // [rad]
        const double max_alpha      = 7.0*M_PI/180.0;   // [rad]

        const double length         = 4.635;            // [m]
        const double width          = 1.890;            // [m]

        const double k_R            = 0.015;            // rolling resistance coefficient [-]

        const double rho            = 1.225;            // air density [kg/m^3]
        const double Cd             = 0.32;             // drag coefficient [-]

        const double Cf             = 154700.0;         // tire cornering stiffness [N/rad]
        const double Cr             = 183350.0;         // tire cornering stiffness [N/rad]
        const double Fyf_max        = 6200.0;           // lateral tire force [N]
        const double Fyr_max        = 6200.0;           // lateral tire force [N]

        const double Mass           = 1319.91;          // [kg]
        const double Inertia        = 2093.38;          // inertia [kg*m^2]
        const double Af             = 1.55*1.8;         // frontal area [m^2]
        const double L_f            = 1.302;            // length from CG to front axle [m]
        const double L_r            = 1.398;            // length from CG to rear axle [m]
        const double wheel_base     = L_f + L_r;        // wheel base [m]

        const double I_w            = 0.3312;           // inertia of wheel [kg*m^2]
        const double I_t            = 0.060;            // inertia of transmission [kg*m^2]
        const double I_m            = 0.015;            // inertia of motor [kg*m^2]

        const double r_eff          = 0.305;            // effective radius [m]
        const double gear_ratio     = 1.0 / 7.98;       // gear ratio [-]

        const double accel_const    = 293.1872055;      // T_motor = accel(0~1)*accel_const
        const double brake_const    = 4488.075;         // T_Brake = brake(0~1)*brake_const
    } // namespace vehicle_param

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        std::string id;
        double x{0.0};
        double y{0.0};
        double pitch{0.0};
        double yaw{0.0};
        double yaw_rate{0.0};
        double slip_angle{0.0};
        double velocity{0.0};
        double length{vehicle_param::length};
        double width{vehicle_param::width};
    } VehicleState;
    
    typedef struct {
        double accel{0.0};
        double brake{0.0};
        double steering{0.0};
    } VehicleCommand;
} // namespace interface

#endif // __INTERFACE_VEHICLE_STATE_HPP__