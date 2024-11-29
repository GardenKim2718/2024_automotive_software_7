/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_node.cpp
 * @brief     simulate vehicle
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#include "vehicle/vehicle_simulator_node.hpp"

Vehicle::Vehicle(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options), tf2_broadcaster_(this) {
        
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameter init
    this->declare_parameter("vehicle/ns", "");
    this->declare_parameter("vehicle/loop_rate_hz", 100.0);
    this->declare_parameter("vehicle/init_x", 0.0);
    this->declare_parameter("vehicle/init_y", 0.0);
    this->declare_parameter("vehicle/init_pitch", 0.0);
    this->declare_parameter("vehicle/init_yaw", 0.0);
    this->declare_parameter("vehicle/init_vel", 0.0);
    this->declare_parameter("vehicle/uphill_slope", 10.0);
    this->declare_parameter("vehicle/downhill_slope", -10.0);
    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "init_x: %f", cfg_.init_x);
    RCLCPP_INFO(this->get_logger(), "init_y: %f", cfg_.init_y);
    RCLCPP_INFO(this->get_logger(), "init_pitch: %f", cfg_.init_pitch);
    RCLCPP_INFO(this->get_logger(), "init_yaw: %f", cfg_.init_yaw);
    RCLCPP_INFO(this->get_logger(), "init_vel: %f", cfg_.init_vel);
    RCLCPP_INFO(this->get_logger(), "uphill_slope: %f", cfg_.uphill_slope);
    RCLCPP_INFO(this->get_logger(), "downhill_slope: %f", cfg_.downhill_slope);

    // Subscriber init
    s_vehicle_command_ = this->create_subscription<ad_msgs::msg::VehicleCommand> (
        "vehicle_command", qos_profile, std::bind(&Vehicle::CallbackVehicleCommand, this, std::placeholders::_1));
    s_mission_ = this->create_subscription<ad_msgs::msg::Mission> (
        "mission", qos_profile, std::bind(&Vehicle::CallbackMission, this, std::placeholders::_1));

    // Publisher init
    p_vehicle_state_ = this->create_publisher<ad_msgs::msg::VehicleState> (
        "vehicle_state", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); }); 

    // Algorithm init
    alg_vehicle_simulator_ = std::make_unique<VehicleSimulatorAlgorithm>(cfg_);
    time_prev_ = this->now().seconds();

    RCLCPP_WARN_STREAM(this->get_logger(), "Initialize node (Period: " << cfg_.loop_rate_hz << " Hz)");
}

Vehicle::~Vehicle() {}

void Vehicle::ProcessParams() {
    this->get_parameter("vehicle/ns", cfg_.vehicle_namespace);
    this->get_parameter("vehicle/loop_rate_hz", cfg_.loop_rate_hz);
    this->get_parameter("vehicle/init_x", cfg_.init_x);
    this->get_parameter("vehicle/init_y", cfg_.init_y);
    this->get_parameter("vehicle/init_pitch", cfg_.init_pitch);
    this->get_parameter("vehicle/init_yaw", cfg_.init_yaw);  
    this->get_parameter("vehicle/init_vel", cfg_.init_vel);
    this->get_parameter("vehicle/uphill_slope", cfg_.uphill_slope);
    this->get_parameter("vehicle/downhill_slope", cfg_.downhill_slope);
}

void Vehicle::Run() {
    auto current_time = this->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::VehicleCommand vehicle_command; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_command_);
        vehicle_command = i_vehicle_command_;
    }

    interface::Mission mission; {
        std::lock_guard<std::mutex> lock(mutex_mission_);
        mission = i_mission_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    double curr_simulation_time = current_time.seconds();
    double time_dt = curr_simulation_time - time_prev_;
    if (time_dt <= 0.0) {
        return;
    }
    time_prev_ = curr_simulation_time;

    // Simulate Vehicle Model
    interface::VehicleState updated_vehicle_state;
    if (b_is_vehicle_command_ == true && b_is_mission_ == true) {
        updated_vehicle_state = alg_vehicle_simulator_->RunAlgorithm(vehicle_command, mission, time_dt, cfg_);
    }
    else {
        updated_vehicle_state = alg_vehicle_simulator_->GetInitialVehicleState();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    p_vehicle_state_->publish(ros2_bridge::UpdateVehicleState(updated_vehicle_state));
    tf2_broadcaster_.sendTransform(ros2_bridge::UpdateVehicleTransform(current_time, updated_vehicle_state, cfg_.vehicle_namespace));
}

int main(int argc, char **argv) {
    std::string node_name = "vehicle";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vehicle>(node_name));
    rclcpp::shutdown();

    return 0;
}