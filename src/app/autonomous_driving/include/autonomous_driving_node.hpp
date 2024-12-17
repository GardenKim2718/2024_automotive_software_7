#ifndef AUTONOMOUS_DRIVING_NODE_HPP
#define AUTONOMOUS_DRIVING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ad_msgs/msg/vehicle_command.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/mission.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <vector>
#include <mutex>

class AutonomousDriving : public rclcpp::Node {
public:
    explicit AutonomousDriving(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AutonomousDriving();

private:
    // ROS Callbacks
    void CallbackManualInput(const ad_msgs::msg::VehicleCommand::SharedPtr msg);
    void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg);
    void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg);
    void CallbackMission(const ad_msgs::msg::Mission::SharedPtr msg);

    // Core Function
    void Run();
    void ProcessParams();

    // DBSCAN Clustering & SG Filter
    double weightedEuclideanDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b, double x_weight);
    std::vector<std::vector<geometry_msgs::msg::Point>> dbscanClustering(const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight);
    std::vector<double> generateSavitzkyGolayKernel(int window_size, int poly_order);
    std::vector<double> applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel);

    // Node Parameters
    double param_pp_kd_, param_pp_kv_, param_pp_kc_; // Pure Pursuit parameters
    double param_pid_kp_, param_pid_ki_, param_pid_kd_; // PID parameters
    double eps, x_weight;
    int min_points, window_size, poly_order;

    // State Flags
    bool b_is_manual_input_ = false;
    bool b_is_simulator_on_ = false;
    bool b_is_lane_points_ = false;
    bool b_is_mission_ = false;
    bool b_trigger_merge = false;
    bool b_is_merge_safe = true;

    // Vehicle States & Mission
    ad_msgs::msg::VehicleCommand i_manual_input_;
    ad_msgs::msg::VehicleState i_vehicle_state_;
    ad_msgs::msg::LanePointData i_lane_points_;
    ad_msgs::msg::Mission i_mission_;

    // Control Variables
    double lateral_error = 0.0;
    double last_lateral_error = 0.0;
    double speed_error_integral_ = 0.0;
    double speed_error_prev_ = 0.0;

    const double max_steering_angle = 0.5; // Maximum steering angle in radians
    const double stability_factor = 0.8;   // Exponential smoothing factor
    const double interval = 0.1;           // Control loop interval in seconds
    const double integral_max = 10.0;      // Maximum integral windup
    const double steering_threshold = 0.3; // Steering threshold for speed reduction
    const double safe_distance = 10.0;     // Minimum safe distance for collision avoidance
    const double merge_speed = 5.0;        // Speed during lane merge
    const double min_speed = 3.0;          // Minimum allowed speed
    const double icy_speed = 7.0;          // Safe speed for icy roads
    double prev_lane_center = 0.0;
    double target_lane_center = 0.0;

    // ROS 2 Publishers and Subscribers
    rclcpp::Subscription<ad_msgs::msg::VehicleCommand>::SharedPtr s_manual_input_;
    rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
    rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr s_lane_points_;
    rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_;

    rclcpp::Publisher<ad_msgs::msg::VehicleCommand>::SharedPtr p_vehicle_command_;

    // Timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // Mutexes for Thread Safety
    std::mutex mutex_manual_input_;
    std::mutex mutex_vehicle_state_;
    std::mutex mutex_lane_points_;
    std::mutex mutex_mission_;
};

#endif // AUTONOMOUS_DRIVING_NODE_HPP
