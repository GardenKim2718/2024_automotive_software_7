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
 */

/**
 * @date
 * 2024-12-06 Seokhui Han updated variable names
 * 2024-12-06 Chungwon Kim updated code to abide coding guideline
 */

#include "autonomous_driving_node.hpp"

AutonomousDriving::AutonomousDriving(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    this->declare_parameter("autonomous_driving/ns", "");
    if (!this->get_parameter("autonomous_driving/ns", vehicle_namespace_param_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
        vehicle_namespace_param_ = "";
    }
    else {
        RCLCPP_INFO(this->get_logger(), "vehicle_namespace_param_: %s", vehicle_namespace_param_.c_str());
    }
    
    this->declare_parameter("autonomous_driving/loop_rate_hz", 100.0);
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);    
    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////
    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", config_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", config_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "use_manual_inputs: %d", config_.use_manual_inputs);
    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////

    // Subscriber init
    manual_input_sub_ = this->create_subscription<ad_msgs::msg::VehicleCommand>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    vehicle_state_sub_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    lane_points_sub_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile, std::bind(&AutonomousDriving::CallbackLanePoints, this, std::placeholders::_1));
    mission_sub_ = this->create_subscription<ad_msgs::msg::Mission>(
        "mission", qos_profile, std::bind(&AutonomousDriving::CallbackMission, this, std::placeholders::_1));

    // Publisher init
    vehicle_command_pub_ = this->create_publisher<ad_msgs::msg::VehicleCommand>(
        "vehicle_command", qos_profile);
    driving_way_pub_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    poly_lanes_pub_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // Timer init
    node_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / config_.loop_rate_hz)),
        [this]() { this->Run(); }); 
}

AutonomousDriving::~AutonomousDriving() {}

void AutonomousDriving::ProcessParams() {
    this->get_parameter("autonomous_driving/ns", config_.vehicle_namespace);
    this->get_parameter("autonomous_driving/loop_rate_hz", config_.loop_rate_hz);
    this->get_parameter("autonomous_driving/use_manual_inputs", config_.use_manual_inputs);
    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////
}

void AutonomousDriving::Run() {
    auto current_time = this->now(); // current_time : Current time of the system, unit: seconds
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");
    ProcessParams();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (config_.use_manual_inputs == true) {
        if (is_manual_input_received_ == false) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Manual Input ...");
            return;
        }
    }

    if (is_simulator_on_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Vehicle State ...");
        return;
    }

    if (is_lane_points_received_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Lane Points ...");
        return;
    }

    if (is_mission_received_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Mission ...");
        return;
    }

    // Copy shared data from subscribers to local variables
    interface::VehicleCommand manual_input_data; { // manual_input_data : Command received from manual input control
        if (config_.use_manual_inputs == true) {
            std::lock_guard<std::mutex> lock(manual_input_mutex_); // lock : Mutex to protect access to manual_input_
            manual_input_data = manual_input_; // Assign manual input command
        }
    }

    interface::VehicleState current_vehicle_state; { // current_vehicle_state : Current state of the vehicle (position, velocity, etc.)
        std::lock_guard<std::mutex> lock(vehicle_state_mutex_); // lock : Mutex to protect access to vehicle_state_
        current_vehicle_state = vehicle_state_; // Assign current vehicle state
    }

    interface::Lane current_lane_points; { // current_lane_points : Lane points detected by the vehicle's sensors
        std::lock_guard<std::mutex> lock(lane_points_mutex_); // lock : Mutex to protect access to lane_points_
        current_lane_points = lane_points_; // Assign lane points
    }

    interface::Mission current_mission; { // current_mission : Mission details (e.g., destination, speed limits)
        std::lock_guard<std::mutex> lock(mission_mutex_); // lock : Mutex to protect access to mission_
        current_mission = mission_; // Assign mission data
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    ////////////////////// TODO //////////////////////
    // TODO: Add polyfit lane algorithm
    interface::PolyfitLanes poly_lanes;
    poly_lanes.frame_id = vehicle_namespace_param_ + "/body";

    // TODO: Add find driving way algorithm
    interface::PolyfitLane  driving_way;
    driving_way.frame_id = vehicle_namespace_param_ + "/body";

    // TODO: Add lateral and longitudinal control algorithm
    interface::VehicleCommand command_to_vehicle;

    //////////////////////////////////////////////////
    // Lane detection and classification
    std::vector<geometry_msgs::msg::Point> filtered_lane_points; // filtered_lane_points : List of lane points after filtering

    // Filter and store all lane points
    for (const auto& point : current_lane_points.point) {
        filtered_lane_points.push_back(point); // Add each point to the filtered list
    }

    RCLCPP_INFO(this->get_logger(), "Number of filtered points: %zu", filtered_lane_points.size());

    // Sort points based on x-coordinates
    std::sort(filtered_lane_points.begin(), filtered_lane_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });

    // Classify lane points into right and left lanes
    const double lane_threshold = 2.0; // lane_threshold : Threshold for classifying points as left or right lane

    std::vector<geometry_msgs::msg::Point> right_lane_points, left_lane_points; // right_lane_points : Points belonging to the right lane, left_lane_points : Points belonging to the left lane
    geometry_msgs::msg::Point last_left_point, last_right_point; // last_left_point : Last point in the left lane, last_right_point : Last point in the right lane

    // Initial classification of lane points
    for (const auto& point : filtered_lane_points) {
        if (point.x > -1.0 && point.x < 2.0) { // Check if point is within the valid x range
            if (right_lane_points.empty() && point.y < 0) {
                right_lane_points.push_back(point); // Assign to right lane
                last_right_point = point; // Update last right point
            } else if (left_lane_points.empty() && point.y > 0) {
                left_lane_points.push_back(point); // Assign to left lane
                last_left_point = point; // Update last left point
            }
            // Break if initial points for both lanes are found
            if (!right_lane_points.empty() && !left_lane_points.empty()) {
                break;
            }
        }
    }

    // Assign subsequent points to lanes based on proximity
    for (const auto& point : filtered_lane_points) {
        // Check if this point belongs to the current right lane
        if (std::abs(point.y - last_right_point.y) < lane_threshold) {
            right_lane_points.push_back(point); // Add to right lane
            last_right_point = point; // Update the last right lane point
        } 
        // Check if this point belongs to the current left lane
        else if (std::abs(point.y - last_left_point.y) < lane_threshold) {
            left_lane_points.push_back(point); // Add to left lane
            last_left_point = point; // Update the last left lane point
        }
    }

    RCLCPP_INFO(this->get_logger(), "Right lane points: %zu", right_lane_points.size());
    RCLCPP_INFO(this->get_logger(), "Left lane points: %zu", left_lane_points.size());

    //    With divided points, you can curve fit the lane and find the left, right lane.
    //    The generated left and right lane should be stored in the "poly_lanes".
    //    If you do so, the Display node will visualize the lanes.

    // Perform polynomial fitting for left lane
    Eigen::MatrixXd A(left_lane.size(), 4);  // A : Matrix for fitting the cubic polynomial for the left lane
    Eigen::VectorXd b(left_lane.size()); // b : Vector for the y-coordinates of the left lane points

    // Fill the matrix A and vector b with left lane points
    for (size_t i = 0; i < left_lane.size(); ++i) {

        double x = left_lane[i].x;

        b(i) = left_lane[i].y;

        A(i, 0) = std::pow(x, 3);
        A(i, 1) = std::pow(x, 2);
        A(i, 2) = x;
        A(i, 3) = 1.0;
    }

    // Calculate the coefficients for the left lane using pseudo inverse
    Eigen::VectorXd left_coeffs = A.completeOrthogonalDecomposition().solve(b); // left_coeffs : Coefficients for the left lane polynomial

    // Perform polynomial fitting for right lane
    Eigen::MatrixXd A_right(right_lane.size(), 4); // A_right : Matrix for fitting the cubic polynomial for the right lane
    Eigen::VectorXd b_right(right_lane.size()); // b_right : Vector for the y-coordinates of the right lane points

    for (size_t i = 0; i < right_lane.size(); ++i) {
        double x = right_lane[i].x;
        b_right(i) = right_lane[i].y;
        A_right(i, 0) = std::pow(x, 3);
        A_right(i, 1) = std::pow(x, 2);
        A_right(i, 2) = x;
        A_right(i, 3) = 1.0;
    }

    Eigen::VectorXd right_coeffs = A_right.completeOrthogonalDecomposition().solve(b_right); // right_coeffs : Coefficients for the right lane polynomial

    // Store polynomial coefficients in PolyfitLaneData
    ad_msgs::msg::PolyfitLaneData left_polyline, right_polyline;

    // Set IDs or any unique identifiers for each lane fit
    left_polyline.frame_id = param_vehicle_namespace_ + "/body"; // left_polyline.frame_id : Frame of reference for left lane polyline
    left_polyline.id = "1"; // left_polyline.id : Unique identifier for the left lane
    right_polyline.frame_id = param_vehicle_namespace_ + "/body"; // right_polyline.frame_id : Frame of reference for right lane polyline
    right_polyline.id = "2"; // right_polyline.id : Unique identifier for the right lane

    // Assign coefficients to the left and right lane polylines
    left_polyline.a0 = left_coeffs(3);
    left_polyline.a1 = left_coeffs(2);
    left_polyline.a2 = left_coeffs(1);
    left_polyline.a3 = left_coeffs(0);

    right_polyline.a0 = right_coeffs(3);
    right_polyline.a1 = right_coeffs(2);
    right_polyline.a2 = right_coeffs(1);
    right_polyline.a3 = right_coeffs(0);

    // Add the polylines to the poly_lanes' polyfitlanes list
    poly_lanes.polyfitlanes.push_back(left_polyline);
    poly_lanes.polyfitlanes.push_back(right_polyline);

    // Generate center driving lane as the average of left and right lanes
    //    The generated center line should be stored in the "driving_way".
    //    If you do so, the Display node will visualize the center line.
    driving_way.a3 = (left_coeffs(0) + right_coeffs(0)) / 2.0;
    driving_way.a2 = (left_coeffs(1) + right_coeffs(1)) / 2.0;
    driving_way.a1 = (left_coeffs(2) + right_coeffs(2)) / 2.0;
    driving_way.a0 = (left_coeffs(3) + right_coeffs(3)) / 2.0;
    
    RCLCPP_INFO(this->get_logger(), "Driving_way - a3: %f, a2: %f, a1: %f, a0: %f", 
    driving_way.a3, driving_way.a2, driving_way.a1, driving_way.a0);

    if (cfg_.use_manual_inputs == false) {
        // Lateral control: Calculate steering angle based on Pure Pursuit
        //    You can tune your controller using the ros parameter.
        //    We provide the example of 'Pure Pursuit' parameters, so you can edit and use them.
        double alpha = 0.5; // alpha: Smoothing factor for lateral error low-pass-filter
        double limit_speed = mission.speed_limit

        // Compute the original lateral_error
        double lateral_error = driving_way.a0 * std::pow(param_m_Lookahead_distance, 3)
                + driving_way.a1 * std::pow(param_m_Lookahead_distance, 2)
                + driving_way.a2 * param_m_Lookahead_distance
                + driving_way.a3;

        // stop if out of track
        if (filtered_points.size() == 0){
            vehicle_command.accel = 0.0;
            vehicle_command.brake = 1.0;
            lateral_error = last_lateral_error;        //maintain last steering
        }

        // Apply the low-pass filter
        lateral_error = alpha * lateral_error + (1.0 - alpha) * last_lateral_error;
        last_lateral_error = lateral_error;

        // Use filtered_e_ for steering calculation
        double steering = atan((2 * param_pp_kd_ * e_) / (param_pp_kv_ * vehicle_state.velocity + param_pp_kc_));
        vehicle_command.steering = std::clamp(steering, -max_steering_angle, max_steering_angle);

        // Longitudinal control: Calculate acceleration/brake based on PID
        // Extract ego vehicle's position
        double ego_x = vehicle_state.x;
        double ego_y = vehicle_state.y;

        // obstacle distance calculation
        double min_distance = 100.0;
        double obs_velocity = 20.0;

        if (obstacles.obstacles.size() > 1) {
            const auto& obstacle = obstacles.obstacles[1];
            // Calculate the distance to the obstacle
            double delta_x = ego_x - obstacle.x;
            double delta_y = ego_y - obstacle.y;
            min_distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
            obs_velocity = obstacle.velocity;
        }

        RCLCPP_INFO(this->get_logger(), "Min distance: %.2f", min_distance);

        double target_speed = 0;

        if (min_distance < 20.0) {  // Collision avoidance scenario
            // Calculate a safe decel/accel based on how close the vehicle is to the obstacle
            if (min_distance < safe_distance){
                target_speed = std::clamp( obs_velocity - 3.5, min_speed, limit_speed - 0.05);
            } else{
                target_speed = std::clamp( obs_velocity + 2.0 , min_speed ,limit_speed - 0.05);
            }

        }   else{                                 // regular longitudinal control(PID + anti-windup)
            target_speed = limit_speed - vehicle_state.velocity - 0.05 ;
        }

        if (steering > steering_threshold || steering < -steering_threshold) {
            target_speed = min_speed;
        }

        speed_error = target_speed - vehicle_state.velocity;

        // interval = (current_time.seconds() - last_time.seconds());  // Calculate time interval between controls in seconds
        speed_error_integral_ = std::clamp(speed_error_integral_ + speed_error * interval, -integral_max, integral_max);

        double diff = (speed_error - speed_error_prev_) / interval;

        double pidvalue_ = param_pid_kp_ * speed_error + param_pid_kd_ * diff + param_pid_ki_ * speed_error_integral_;   // PID control
        pidvalue_ = std::clamp(pidvalue_, -1.0, 1.0);

        vehicle_command.accel = 0.0;
        vehicle_command.brake = 0.0;

        if (pidvalue_ > 0) {
            vehicle_command.accel = pidvalue_;
            vehicle_command.brake = 0.0;
        }
        else{
            vehicle_command.accel = 0.0;
            vehicle_command.brake = -pidvalue_;
        }

        RCLCPP_INFO(this->get_logger(), "Vehicle Command - Accel: %.2f, Brake: %.2f, Steering: %.2f",
             vehicle_command.accel, vehicle_command.brake, vehicle_command.steering);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Update output
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        o_driving_way_ = driving_way;
        o_poly_lanes_ = poly_lanes;
        o_vehicle_command_ = vehicle_command;
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Publish output
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        Publish(current_time);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Publish output
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        p_vehicle_command_->publish(ros2_bridge::UpdateVehicleCommand(vehicle_command));
        p_driving_way_->publish(ros2_bridge::UpdatePolyfitLane(driving_way));
        p_poly_lanes_->publish(ros2_bridge::UpdatePolyfitLanes(poly_lanes));
    }

    else {
        vehicle_command = manual_input;
    }
}

int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name));
    rclcpp::shutdown();
    return 0;
}
