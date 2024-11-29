/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_node.cpp
 * @brief     simulate lane detection
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#include "lane_detection/lane_detection_node.hpp"

LaneDetection::LaneDetection(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameter init
    this->declare_parameter("lane_detection/ns", "");
    this->declare_parameter("lane_detection/loop_rate_hz", 30.0);
    this->declare_parameter("lane_detection/ROI_front", 20.0);
    this->declare_parameter("lane_detection/ROI_rear", 10.0);
    this->declare_parameter("lane_detection/ROI_left", 3.0);
    this->declare_parameter("lane_detection/ROI_right", 3.0);
    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "ROI_front: %f", cfg_.ROI_front);
    RCLCPP_INFO(this->get_logger(), "ROI_rear: %f", cfg_.ROI_rear);
    RCLCPP_INFO(this->get_logger(), "ROI_left: %f", cfg_.ROI_left);
    RCLCPP_INFO(this->get_logger(), "ROI_right: %f", cfg_.ROI_right);
    
    std::string dir(getenv("PWD"));
    std::string csv_path("/resources/csv/simulation_lane");
    cfg_.ref_csv_path = dir + csv_path;
    RCLCPP_INFO(this->get_logger(), "ref_csv_path: %s", cfg_.ref_csv_path.c_str());

    // Subscriber init
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState> (
        "vehicle_state", qos_profile, std::bind(&LaneDetection::CallbackVehicleState, this, std::placeholders::_1));

    // Publisher init
    p_csv_lanes_ = this->create_publisher<ad_msgs::msg::LanePointDataArray>(
        "csv_lanes", qos_profile);
    p_roi_lanes_ = this->create_publisher<ad_msgs::msg::LanePointDataArray>(
        "ROI_lanes", qos_profile);
    p_lane_points_ = this->create_publisher<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); }); 
        
    // Algorithm init
    alg_lane_detection_ = std::make_unique<LaneDetectionAlgorithm>(cfg_);
    
    RCLCPP_WARN_STREAM(this->get_logger(), "Initialize node (Period: " << cfg_.loop_rate_hz << " Hz)");
}

LaneDetection::~LaneDetection() {}

void LaneDetection::ProcessParams() {
    this->get_parameter("lane_detection/ns", cfg_.vehicle_namespace);  
    this->get_parameter("lane_detection/loop_rate_hz", cfg_.loop_rate_hz);
    this->get_parameter("lane_detection/ROI_front", cfg_.ROI_front);
    this->get_parameter("lane_detection/ROI_rear", cfg_.ROI_rear);
    this->get_parameter("lane_detection/ROI_left", cfg_.ROI_left);
    this->get_parameter("lane_detection/ROI_right", cfg_.ROI_right);  
}

void LaneDetection::Run() {
    auto current_time = this->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    if (b_is_simulator_on_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Vehicle State ...");
        return;
    }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::Lanes csv_lanes = alg_lane_detection_->GetRefLanes();
    interface::Lanes roi_lanes = alg_lane_detection_->RunAlgorithm(vehicle_state, cfg_);
    interface::Lane shuffled_roi_lane = alg_lane_detection_->GetRefRandomLane();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if ((current_time.seconds() - time_prev_csv_lanes_) > 5.0) {
        time_prev_csv_lanes_ = current_time.seconds();
        p_csv_lanes_->publish(ros2_bridge::UpdateCsvLanes(csv_lanes));
    }

    p_roi_lanes_->publish(ros2_bridge::UpdateROILanes(roi_lanes));
    p_lane_points_->publish(ros2_bridge::UpdateLanePoints(shuffled_roi_lane, cfg_.vehicle_namespace));
}

int main(int argc, char **argv) {
    std::string node_name = "lane_detection";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneDetection>(node_name));
    rclcpp::shutdown();

    return 0;
}
