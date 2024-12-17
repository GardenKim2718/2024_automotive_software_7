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

    // Parameter Initialization
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("ki", 0.01);
    this->declare_parameter("kd", 0.1);
    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);

    // Subscribers
    s_poly_lanes_ = this->create_subscription<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", 10, std::bind(&AutonomousDriving::CallbackPolyLanes, this, std::placeholders::_1));

    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "vehicle_state", 10, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_ = this->create_subscription<ad_msgs::msg::Mission>(
        "mission", 10, std::bind(&AutonomousDriving::CallbackMission, this, std::placeholders::_1));

    // Publisher
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleCommand>("vehicle_command", 10);

    // Timer for control loop
    this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AutonomousDriving::Run, this));
}

AutonomousDriving::~AutonomousDriving() {}

void AutonomousDriving::CallbackPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg) {
    poly_lanes_ = *msg;
    has_poly_lanes_ = true;
}

void AutonomousDriving::CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
    vehicle_state_ = *msg;
    has_vehicle_state_ = true;
}

void AutonomousDriving::CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg) {
    mission_ = *msg;
    has_mission_ = true;
}

void AutonomousDriving::Run() {
    if (!has_poly_lanes_ || !has_vehicle_state_ || !has_mission_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for all inputs...");
        return;
    }

    // Perform control calculations
    CalculateLateralControl();
    CalculateLongitudinalControl();

    // Publish control commands
    ad_msgs::msg::VehicleCommand vehicle_command;
    vehicle_command.steering = steering_angle_;
    vehicle_command.accel = target_speed_ > vehicle_state_.velocity ? 0.5 : 0.0;
    vehicle_command.brake = target_speed_ <= vehicle_state_.velocity ? 0.5 : 0.0;

    p_vehicle_command_->publish(vehicle_command);

    RCLCPP_INFO(this->get_logger(),
                "Steering: %.2f, Accel: %.2f, Brake: %.2f",
                vehicle_command.steering, vehicle_command.accel, vehicle_command.brake);
}

void AutonomousDriving::CalculateLateralControl() {
    // Use polyfit data for lateral control
    double a0 = poly_lanes_.polyfitlanes[0].a0;
    steering_angle_ = std::clamp(a0 * kp_, -max_steering_angle_, max_steering_angle_);
}

void AutonomousDriving::CalculateLongitudinalControl() {
    // PID control for speed
    double speed_error = mission_.speed_limit - vehicle_state_.velocity;
    speed_error_integral_ = std::clamp(speed_error_integral_ + speed_error * control_interval_, -integral_max_, integral_max_);
    double diff = (speed_error - speed_error_prev_) / control_interval_;

    target_speed_ = kp_ * speed_error + ki_ * speed_error_integral_ + kd_ * diff;
    target_speed_ = std::max(0.0, target_speed_);
    speed_error_prev_ = speed_error;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>("autonomous_driving_node"));
    rclcpp::shutdown();
    return 0;
}
