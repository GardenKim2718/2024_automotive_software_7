/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      autonomous_driving_node.cpp
 * @brief     autonomous driving node
 *
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 *            2024-12-06 updated by Seokhui Han, Chungwon Kim
 *              : Improved coding guidelines and removed unused code.
 */

#include "autonomous_driving_node.hpp"

AutonomousDriving::AutonomousDriving(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    this->declare_parameter("autonomous_driving/ns", "");
    this->declare_parameter("autonomous_driving/loop_rate_hz", 100.0);
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);
       
    // Custom Parameters
    this->declare_parameter<double>("autonomous_driving/param_pp_kd", 1.0);
    this->declare_parameter<double>("autonomous_driving/param_pp_kv", 0.05);
    this->declare_parameter<double>("autonomous_driving/param_pp_kc", 0.5);
    this->declare_parameter<double>("autonomous_driving/param_pid_kp", 5.0);
    this->declare_parameter<double>("autonomous_driving/param_pid_ki", 0.002);
    this->declare_parameter<double>("autonomous_driving/param_pid_kd", 0.01);
    this->declare_parameter<double>("autonomous_driving/eps", 5.0);
    this->declare_parameter<double>("autonomous_driving/x_weight", 0.05);
    this->declare_parameter<int>("autonomous_driving/min_points", 3);
    this->declare_parameter<int>("autonomous_driving/window_size", 7);
    this->declare_parameter<int>("autonomous_driving/poly_order", 2);

    this->get_parameter("autonomous_driving/param_pp_kd", param_pp_kd_);
    this->get_parameter("autonomous_driving/param_pp_kv", param_pp_kv_);
    this->get_parameter("autonomous_driving/param_pp_kc", param_pp_kc_);
    this->get_parameter("autonomous_driving/param_pid_kp", param_pid_kp_);
    this->get_parameter("autonomous_driving/param_pid_ki", param_pid_ki_);
    this->get_parameter("autonomous_driving/param_pid_kd", param_pid_kd_);
    this->get_parameter("autonomous_driving/window_size", window_size);
    this->get_parameter("autonomous_driving/poly_order", poly_order);
    this->get_parameter("autonomous_driving/eps", eps);
    this->get_parameter("autonomous_driving/x_weight", x_weight);
    this->get_parameter("autonomous_driving/min_points", min_points);

    //////////////////////////////////////////////////
    ProcessParams();

    // Subscriber init
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleCommand>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile, std::bind(&AutonomousDriving::CallbackLanePoints, this, std::placeholders::_1));
    s_mission_ = this->create_subscription<ad_msgs::msg::Mission>(
        "mission", qos_profile, std::bind(&AutonomousDriving::CallbackMission, this, std::placeholders::_1));

    // Publisher init
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleCommand>(
        "vehicle_command", qos_profile);
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); }); 
}

AutonomousDriving::~AutonomousDriving() {}

void AutonomousDriving::ProcessParams() {
    this->get_parameter("autonomous_driving/ns", cfg_.vehicle_namespace);
    this->get_parameter("autonomous_driving/loop_rate_hz", cfg_.loop_rate_hz);
    this->get_parameter("autonomous_driving/use_manual_inputs", cfg_.use_manual_inputs);
}

void AutonomousDriving::CallbackManualInput(const ad_msgs::msg::VehicleCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_manual_input_);
    i_manual_input_ = *msg;
    b_is_manual_input_ = true;
}

void AutonomousDriving::CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
    i_vehicle_state_ = *msg;
    b_is_simulator_on_ = true;
}

void AutonomousDriving::CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_lane_points_);
    i_lane_points_ = *msg;
    b_is_lane_points_ = true;
}

void AutonomousDriving::CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_mission_);
    i_mission_ = *msg;
    b_is_mission_ = true;
}

void AutonomousDriving::Run() {
    if (!b_is_lane_points_ || !b_is_simulator_on_ || !b_is_mission_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for inputs...");
        return;
    }

    // Process vehicle control, lane fitting, and mission planning as before.
    RCLCPP_INFO(this->get_logger(), "Running Autonomous Driving Logic...");

    ////////////////////// Control Logic //////////////////////
    interface::VehicleCommand vehicle_command;

    // ----- Lateral Control: Steering -----
    double steering = atan((2 * param_pp_kd_ * lateral_error) /
                           (param_pp_kv_ * i_vehicle_state_.velocity + param_pp_kc_));
    vehicle_command.steering = std::clamp(steering, -max_steering_angle, max_steering_angle);

    // ----- Longitudinal Control: Speed PID -----
    double target_speed = std::min(i_mission_.speed_limit, 15.0); // Limit speed to safe value
    double speed_error = target_speed - i_vehicle_state_.velocity;
    speed_error_integral_ = std::clamp(speed_error_integral_ + speed_error * interval, -integral_max, integral_max);

    double diff = (speed_error - speed_error_prev_) / interval;
    double pid_output = param_pid_kp_ * speed_error + param_pid_ki_ * speed_error_integral_ + param_pid_kd_ * diff;

    // Apply acceleration or brake
    if (pid_output > 0) {
        vehicle_command.accel = std::clamp(pid_output, 0.0, 1.0);
        vehicle_command.brake = 0.0;
    } else {
        vehicle_command.accel = 0.0;
        vehicle_command.brake = std::clamp(-pid_output, 0.0, 1.0);
    }

    // ----- Merge Safety Check -----
    if (b_trigger_merge && !b_is_merge_safe) {
        RCLCPP_WARN(this->get_logger(), "Unsafe merge detected, applying full brake!");
        vehicle_command.accel = 0.0;
        vehicle_command.brake = 1.0;
    }

    // Log control commands
    RCLCPP_INFO(this->get_logger(),
                "Control Output -> Steering: %.3f, Accel: %.3f, Brake: %.3f",
                vehicle_command.steering, vehicle_command.accel, vehicle_command.brake);

    // Publish vehicle command
    p_vehicle_command_->publish(vehicle_command);
}
