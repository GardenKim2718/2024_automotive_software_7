/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      mission_manager_node.cpp
 * @brief     generate mission events node
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *            2024-11-06 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#include "mission_manager_node.hpp"

ScenarioRunner::ScenarioRunner(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
        
    // QoS init    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Parameter init
    this->declare_parameter("mission_manager/ns", "");
    this->declare_parameter("mission_manager/loop_rate_hz", 100.0);
    this->declare_parameter("mission_manager/id_merging_lane", 0);
    this->declare_parameter("mission_manager/num_of_lane", 3);
    this->declare_parameter("mission_manager/num_of_merging_vehicles", 3);
    this->declare_parameter("mission_manager/detection_range", 15.0);
    this->declare_parameter("mission_manager/object_max_accel", 3.0);
    this->declare_parameter("mission_manager/merging_section_length", 25.0);
    this->declare_parameter("mission_manager/merging_interval", 10.0);
    this->declare_parameter("mission_manager/merging_speed", 30.0);
    this->declare_parameter("mission_manager/merging_target_speed", 50.0);
    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "id_merging_lane: %d", cfg_.id_merging_lane);
    RCLCPP_INFO(this->get_logger(), "num_of_lane: %d", cfg_.num_of_lane);
    RCLCPP_INFO(this->get_logger(), "num_of_merging_vehicles: %d", cfg_.num_of_merging_vehicles);
    RCLCPP_INFO(this->get_logger(), "detection_range: %f", cfg_.detection_range);
    RCLCPP_INFO(this->get_logger(), "object_max_accel: %f", cfg_.object_max_accel);
    RCLCPP_INFO(this->get_logger(), "merging_section_length: %f", cfg_.merging_section_length);
    RCLCPP_INFO(this->get_logger(), "merging_interval: %f", cfg_.merging_interval);
    RCLCPP_INFO(this->get_logger(), "merging_speed: %f", cfg_.merging_speed);
    RCLCPP_INFO(this->get_logger(), "merging_target_speed: %f", cfg_.merging_target_speed);
    
    std::string dir(getenv("PWD"));
    std::string csv_path("/resources/csv/evaluation_lane");
    cfg_.ref_csv_path = dir + csv_path;
    RCLCPP_INFO(this->get_logger(), "ref_csv_path: %s", cfg_.ref_csv_path.c_str());
    
    // Subscriber init
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState> (
        "vehicle_state", qos_profile, std::bind(&ScenarioRunner::CallbackVehicleState, this, std::placeholders::_1));
    
    // Publisher init
    p_detected_mission_ = this->create_publisher<ad_msgs::msg::Mission>(
        "mission", qos_profile);
    p_display_mission_ = this->create_publisher<ad_msgs::msg::MissionDisplay>(
        "mission_display", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); });

    // Algorithm init
    alg_reference_lane_generation_ = std::make_unique<ReferenceLaneGeneration>(cfg_);
    i_ref_lanes_ = alg_reference_lane_generation_->GetReferenceLanes();
    i_id_center_ = alg_reference_lane_generation_->GetCenterLaneId();

    alg_mission_road_condition_ = std::make_unique<MissionRoadCondition>(i_ref_lanes_, i_id_center_, cfg_);
    alg_mission_road_slope_ = std::make_unique<MissionRoadSlope>(i_ref_lanes_, i_id_center_, cfg_);
    RCLCPP_INFO(this->get_logger(), "Road Slope Mission Generation Complete");
    alg_mission_scc_ = std::make_unique<MissionSCC>(i_ref_lanes_, i_id_center_, cfg_); 
    RCLCPP_INFO(this->get_logger(), "SCC Mission Generation Complete");
    alg_mission_speed_limit_ = std::make_unique<MissionSpeedLimit>(i_ref_lanes_, i_id_center_, cfg_);
    RCLCPP_INFO(this->get_logger(), "Speed Limit Mission Generation Complete");
    alg_mission_merge_ = std::make_unique<MissionMerge>(i_ref_lanes_, i_id_center_, cfg_);
    RCLCPP_INFO(this->get_logger(), "Merge Mission Generation Complete");
    time_prev_ = this->now().seconds();

    RCLCPP_WARN_STREAM(this->get_logger(), "Initialize node (Period: " << cfg_.loop_rate_hz << " Hz)");
}

ScenarioRunner::~ScenarioRunner() {}

void ScenarioRunner::ProcessParams() {
    this->get_parameter("mission_manager/ns", cfg_.vehicle_namespace);
    this->get_parameter("mission_manager/loop_rate_hz", cfg_.loop_rate_hz);
    this->get_parameter("mission_manager/id_merging_lane", cfg_.id_merging_lane);
    this->get_parameter("mission_manager/num_of_lane", cfg_.num_of_lane);
    this->get_parameter("mission_manager/num_of_merging_vehicles", cfg_.num_of_merging_vehicles);
    this->get_parameter("mission_manager/detection_range", cfg_.detection_range);
    this->get_parameter("mission_manager/object_max_accel", cfg_.object_max_accel);
    this->get_parameter("mission_manager/merging_section_length", cfg_.merging_section_length);
    this->get_parameter("mission_manager/merging_interval", cfg_.merging_interval);
    this->get_parameter("mission_manager/merging_speed", cfg_.merging_speed);
    this->get_parameter("mission_manager/merging_target_speed", cfg_.merging_target_speed);
}

void ScenarioRunner::Run() {
    auto current_time = this->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (b_is_vehicle_state_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Vehicle State...");
        return;
    }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    double time_dt = current_time.seconds() - time_prev_;
    time_prev_ = current_time.seconds();

    //  Get detected mission
    std::string road_condition                          = alg_mission_road_condition_->RunAlgorithm(vehicle_state, cfg_);
    std::string road_slope                              = alg_mission_road_slope_->RunAlgorithm(vehicle_state, cfg_);
    double speed_limit                                  = alg_mission_speed_limit_->RunAlgorithm(vehicle_state, cfg_);
    std::vector<interface::MissionObject> scc_objects   = alg_mission_scc_->RunAlgorithm(vehicle_state, i_ref_lanes_, time_dt, cfg_);
    std::vector<interface::MissionObject> merge_objects = alg_mission_merge_->RunAlgorithm(vehicle_state, i_ref_lanes_, time_dt, cfg_);

    //  Get all mission to display
    std::vector<interface::MissionRegion> display_road_condition = alg_mission_road_condition_->GetRoadCondition();
    std::vector<interface::MissionRegion> display_road_slope     = alg_mission_road_slope_->GetRoadSlope();
    std::vector<interface::MissionRegion> display_speed_limit    = alg_mission_speed_limit_->GetSpeedLimit();
    std::vector<interface::MissionObject> display_scc_objects    = alg_mission_scc_->GetSCCObjects();
    std::vector<interface::MissionObject> display_merge_objects  = alg_mission_merge_->GetMergeObjects();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    p_detected_mission_->publish(ros2_bridge::UpdateMission(scc_objects, merge_objects, road_condition, road_slope, speed_limit));
    p_display_mission_->publish(ros2_bridge::UpdateMissionDisplay(display_scc_objects, display_merge_objects, display_road_slope, display_road_condition, display_speed_limit));
}

int main(int argc, char **argv) {
    std::string node_name = "mission_manager";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioRunner>(node_name));
    rclcpp::shutdown();

    return 0;
}
