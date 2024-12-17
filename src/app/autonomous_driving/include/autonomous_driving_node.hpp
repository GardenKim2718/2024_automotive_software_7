#ifndef AUTONOMOUS_DRIVING_NODE_HPP
#define AUTONOMOUS_DRIVING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "ad_msgs/msg/vehicle_command.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "ad_msgs/msg/mission.hpp"
#include "ad_msgs/msg/polyfit_lane_data_array.hpp"

class AutonomousDriving : public rclcpp::Node {
public:
    explicit AutonomousDriving(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AutonomousDriving();

private:
    // Callbacks
    void CallbackPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg);
    void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg);
    void CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg);

    // Main loop
    void Run();

    // Control Logic
    void CalculateLateralControl();
    void CalculateLongitudinalControl();

    // Subscribers
    rclcpp::Subscription<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr s_poly_lanes_;
    rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
    rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_;

    // Publisher
    rclcpp::Publisher<ad_msgs::msg::VehicleCommand>::SharedPtr p_vehicle_command_;

    // Variables
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes_;
    ad_msgs::msg::VehicleState vehicle_state_;
    ad_msgs::msg::Mission mission_;

    bool has_poly_lanes_ = false;
    bool has_vehicle_state_ = false;
    bool has_mission_ = false;

    double steering_angle_ = 0.0;
    double target_speed_ = 0.0;

    // Control Parameters
    double kp_, ki_, kd_;
    double speed_error_integral_ = 0.0;
    double speed_error_prev_ = 0.0;
    double max_steering_angle_ = 0.5;
    double control_interval_ = 0.05;
    double integral_max_ = 10.0;
};

#endif // AUTONOMOUS_DRIVING_NODE_HPP
