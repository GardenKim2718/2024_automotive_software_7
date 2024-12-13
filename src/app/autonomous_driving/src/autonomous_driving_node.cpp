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
 * !!!!!!!!!!!!!!!DESCRIBE THE UPDATE LOG YOU MADE!!!!!!!!!!!!!!!!
 * @date
 * 2024-12-06 Seokhui Han updated variable names
 * 2024-12-06 Chungwon Kim updated code to abide coding guideline
 * 2024-12-06 Chungwon Kim code fix and lane_fitting update
 * 2024-12-06 Chungwon Kim testing improved lane fitting algorithm
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
       
    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////
    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "use_manual_inputs: %d", cfg_.use_manual_inputs);
    ////////////////////// TODO //////////////////////
    // TODO: Add more parameters

    //////////////////////////////////////////////////

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
    if (cfg_.use_manual_inputs == true) {
        if (b_is_manual_input_ == false) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Manual Input ...");
            return;
        }
    }

    if (b_is_simulator_on_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Vehicle State ...");
        return;
    }

    if (b_is_lane_points_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Lane Points ...");
        return;
    }

    if (b_is_mission_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Mission ...");
        return;
    }

    // Copy shared data from subscribers to local variables
    interface::VehicleCommand manual_input; { // manual_input_data : Command received from manual input control
        if (cfg_.use_manual_inputs == true) {
            std::lock_guard<std::mutex> lock(mutex_manual_input_); // lock : Mutex to protect access to manual_input_
            manual_input = i_manual_input_; // Assign manual input command
        }
    }

    interface::VehicleState current_vehicle_state; { // current_vehicle_state : Current state of the vehicle (position, velocity, etc.)
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_); // lock : Mutex to protect access to vehicle_state_
        current_vehicle_state = i_vehicle_state_; // Assign current vehicle state
    }

    interface::Lane current_lane_points; { // current_lane_points : Lane points detected by the vehicle's sensors
        std::lock_guard<std::mutex> lock(mutex_lane_points_); // lock : Mutex to protect access to lane_points_
        current_lane_points = i_lane_points_; // Assign lane points
    }

    interface::Mission current_mission; { // current_mission : Mission details (e.g., destination, speed limits)
        std::lock_guard<std::mutex> lock(mutex_mission_); // lock : Mutex to protect access to mission_
        current_mission = i_mission_; // Assign mission data
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    ////////////////////// TODO //////////////////////
    // TODO: Add polyfit lane algorithm
    interface::PolyfitLanes poly_lanes;
    poly_lanes.frame_id = cfg_.vehicle_namespace + "/body";

    // TODO: Add find driving way algorithm
    interface::PolyfitLane driving_way;
    driving_way.frame_id = cfg_.vehicle_namespace + "/body";

    // TODO: Add lateral and longitudinal control algorithm
    interface::VehicleCommand vehicle_command;

    /* >>>>>>> Lane Detection <<<<<<<*/
    std::vector<geometry_msgs::msg::Point> filtered_lane_points; // filtered_lane_points : List of lane points after filtering

    // Filter and store all lane points
    for (const auto& point : current_lane_points.point) {
        geometry_msgs::msg::Point converted_point;
        converted_point.x = point.x; // Assuming interface::Point2D has 'x' field
        converted_point.y = point.y; // Assuming interface::Point2D has 'y' field
        converted_point.z = 0.0;     // Set z value explicitly if needed
        filtered_lane_points.push_back(converted_point); // Add to filtered_lane_points
    }

    RCLCPP_INFO(this->get_logger(), "Number of filtered points: %zu", filtered_lane_points.size());

    // Sort points based on x-coordinates
    std::sort(filtered_lane_points.begin(), filtered_lane_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });

    // Classify lane points into right and left lanes
    std::vector<geometry_msgs::msg::Point> right_lane_points, left_lane_points; // right_lane_points : Points belonging to the right lane, left_lane_points : Points belonging to the left lane
    geometry_msgs::msg::Point last_left_point, last_right_point; // last_left_point : Last point in the left lane, last_right_point : Last point in the right lane

    // Initial classification of lane points
    for (const auto& point : filtered_lane_points) {
        if (point.x > -1.0 && point.x < 5.0) { // Check if point is within the valid x range
            if (right_lane_points.empty() && left_lane_points.empty()){ // if both lanes have not been detected
                if (point.y < 0) {
                    right_lane_points.push_back(point); // Assign to right lane
                    last_right_point = point; // Update last right point
                } else if (point.y > 0) {
                    left_lane_points.push_back(point); // Assign to left lane
                    last_left_point = point; // Update last left point
                }
            }
            else if (!right_lane_points.empty() && left_lane_points.empty()){ // if right_lane is detected but left is undetected
                if (point.y > 0 && point.y > (last_right_point.y + lane_threshold)) {
                    left_lane_points.push_back(point); // Assign to left lane
                    last_left_point = point; // Update last left point
                }
            }
            else if (right_lane_points.empty() && !left_lane_points.empty()){  // if left lane is detected but right is undetected
                if (point.y < 0 && point.y < (last_left_point.y - lane_threshold)) {
                    right_lane_points.push_back(point); // Assign to left lane
                    last_left_point = point; // Update last left point
                }
            }
            else{ 
                break; // Break if initial points for both lanes are found
            }
        }
    }

    // Assign subsequent points to lanes based on proximity
    for (const auto& point : filtered_lane_points) {
        if (point.x > -1.0) { // Check if point is within the valid x range
            // Check if this point belongs to the current right lane
            if ((std::abs(point.y - last_right_point.y) < lane_threshold) && !right_lane_points.empty()) {
                right_lane_points.push_back(point); // Add to right lane
                last_right_point = point; // Update the last right lane point
            }
            // Check if this point belongs to the current left lane
            else if ((std::abs(point.y - last_left_point.y) < lane_threshold) && !left_lane_points.empty()) {
                left_lane_points.push_back(point); // Add to left lane
                last_left_point = point; // Update the last left lane point
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Right lane points: %zu", right_lane_points.size());
    RCLCPP_INFO(this->get_logger(), "Left lane points: %zu", left_lane_points.size());

    /* >>>>>>> Lane Fitting <<<<<<<*/
    //    With classified points, you can curve fit the lane and find the left, right lane.
    //    The generated left and right lane should be stored in the "poly_lanes".
    //    If you do so, the Display node will visualize the lanes.

    // Perform polynomial fitting for left lane
    Eigen::MatrixXd A_left(left_lane_points.size(), 4);  // A_left : Matrix for fitting the cubic polynomial for the left lane
    Eigen::VectorXd b_left(left_lane_points.size()); // b_left : Vector for the y-coordinates of the left lane points
    Eigen::MatrixXd A_right(right_lane_points.size(), 4); // A_right : Matrix for fitting the cubic polynomial for the right lane
    Eigen::VectorXd b_right(right_lane_points.size()); // b_right : Vector for the y-coordinates of the right lane points

    // Fill the matrix A and vector b with left lane points
    for (size_t i = 0; i < left_lane_points.size(); ++i) {
        double x = left_lane_points[i].x;
        b_left(i) = left_lane_points[i].y;
        A_left(i, 0) = 1.0;
        A_left(i, 1) = x;
        A_left(i, 2) = std::pow(x, 2);
        A_left(i, 3) = std::pow(x, 3);
    }

    // Fill the matrix A and vector b with right lane points
    for (size_t i = 0; i < right_lane_points.size(); ++i) {
        double x = right_lane_points[i].x;
        b_right(i) = right_lane_points[i].y;
        A_right(i, 0) = 1.0;
        A_right(i, 1) = x;
        A_right(i, 2) = std::pow(x, 2);
        A_right(i, 3) = std::pow(x, 3);
    }

    // Polynomial fitting (3rd degree)
    Eigen::VectorXd left_coeffs = A_left.completeOrthogonalDecomposition().solve(b_left); // left_coeffs : Coefficients for the left lane polynomial
    Eigen::VectorXd right_coeffs = A_right.completeOrthogonalDecomposition().solve(b_right); // right_coeffs : Coefficients for the right lane polynomial

    // Store polynomial coefficients in PolyfitLaneData
    interface::PolyfitLane left_polyline, right_polyline;
    left_polyline.frame_id = cfg_.vehicle_namespace + "/body"; // left_polyline.frame_id : Frame of reference for left lane polyline
    left_polyline.id = "1"; // left_polyline.id : Unique identifier for the left lane
    right_polyline.frame_id = cfg_.vehicle_namespace + "/body"; // right_polyline.frame_id : Frame of reference for right lane polyline
    right_polyline.id = "2"; // right_polyline.id : Unique identifier for the right lane

    // Assign coefficients to the left and right lane polylines
    left_polyline.a0 = left_coeffs(0);
    left_polyline.a1 = left_coeffs(1);
    left_polyline.a2 = left_coeffs(2);
    left_polyline.a3 = left_coeffs(3);

    right_polyline.a0 = right_coeffs(0);
    right_polyline.a1 = right_coeffs(1);
    right_polyline.a2 = right_coeffs(2);
    right_polyline.a3 = right_coeffs(3);

    // Add the polylines to the poly_lanes' polyfitlanes list
    poly_lanes.polyfitlanes.push_back(left_polyline);
    poly_lanes.polyfitlanes.push_back(right_polyline);

    // Generate center driving lane as the average of left and right lanes
    //    The generated center line should be stored in the "driving_way".
    //    If you do so, the Display node will visualize the center line.
    driving_way.a3 = (left_coeffs(3) + right_coeffs(3)) / 2.0;
    driving_way.a2 = (left_coeffs(2) + right_coeffs(2)) / 2.0;
    driving_way.a1 = (left_coeffs(1) + right_coeffs(1)) / 2.0;
    driving_way.a0 = (left_coeffs(0) + right_coeffs(0)) / 2.0;
    
    // RCLCPP_INFO(this->get_logger(), "Driving_way - a3: %f, a2: %f, a1: %f, a0: %f", driving_way.a3, driving_way.a2, driving_way.a1, driving_way.a0);

    // If using manual input
    if (cfg_.use_manual_inputs == true) {
        vehicle_command = manual_input;
    } // Autonomous driving
    else {
        // Lateral control: Calculate steering angle based on Pure Pursuit
        //    You can tune your controller using the ros parameter.
        //    We provide the example of 'Pure Pursuit' parameters, so you can edit and use them.
        double limit_speed = current_mission.speed_limit;

        // Compute the original lateral_error
        lateral_error = driving_way.a0 * std::pow(param_m_Lookahead_distance, 3)
                + driving_way.a1 * std::pow(param_m_Lookahead_distance, 2)
                + driving_way.a2 * param_m_Lookahead_distance
                + driving_way.a3;

        // Apply the low-pass filter
        lateral_error = alpha * lateral_error + (1.0 - alpha) * last_lateral_error;
        last_lateral_error = lateral_error;

        // Use filtered_e_ for steering calculation
        double steering = atan((2 * param_pp_kd_ * lateral_error) / (param_pp_kv_ * current_vehicle_state.velocity + param_pp_kc_));
        vehicle_command.steering = std::clamp(steering, -max_steering_angle, max_steering_angle);

        // Longitudinal control: Calculate acceleration/brake based on PID
        // Extract ego vehicle's position
        double ego_x = current_vehicle_state.x;
        double ego_y = current_vehicle_state.y;

        // obstacle distance calculation
        double min_distance = 100.0;
        double obs_velocity = 20.0;

        if (current_mission.objects.size() > 1) {
            const auto& obstacle = current_mission.objects[1];
            // Calculate the distance to the obstacle
            double delta_x = ego_x - obstacle.x;
            double delta_y = ego_y - obstacle.y;
            min_distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
            obs_velocity = obstacle.velocity;
        }

        // RCLCPP_INFO(this->get_logger(), "Min distance: %.2f", min_distance);

        double target_speed = 0;

        if (min_distance < 20.0) {  // Collision avoidance scenario
            // Calculate a safe decel/accel based on how close the vehicle is to the obstacle
            if (min_distance < safe_distance){
                target_speed = std::clamp( obs_velocity - 3.5, min_speed, limit_speed - 0.05);
            }
            else{
                target_speed = std::clamp( obs_velocity + 2.0 , min_speed ,limit_speed - 0.05);
            }
        }
        else{  // regular longitudinal control(PID + anti-windup)
            target_speed = limit_speed - current_vehicle_state.velocity - 0.05 ;
        }

        if (steering > steering_threshold || steering < -steering_threshold) {
            target_speed = min_speed;
        }

        RCLCPP_INFO(this->get_logger(), "Target speed: %.2f", target_speed);

        // Longitudinal Control
        speed_error = target_speed - current_vehicle_state.velocity;
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
    }

    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    p_vehicle_command_->publish(ros2_bridge::UpdateVehicleCommand(vehicle_command));
    p_driving_way_->publish(ros2_bridge::UpdatePolyfitLane(driving_way));
    p_poly_lanes_->publish(ros2_bridge::UpdatePolyfitLanes(poly_lanes));
}

int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name));
    rclcpp::shutdown();
    return 0;
}