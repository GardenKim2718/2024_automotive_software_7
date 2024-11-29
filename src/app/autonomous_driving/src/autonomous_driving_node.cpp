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
    auto current_time = this->now();
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

    interface::VehicleCommand manual_input; {
        if (cfg_.use_manual_inputs == true) {
            std::lock_guard<std::mutex> lock(mutex_manual_input_);
            manual_input = i_manual_input_;
        }
    }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }

    interface::Lane lane_points; {
        std::lock_guard<std::mutex> lock(mutex_lane_points_);
        lane_points = i_lane_points_;
    }

    interface::Mission mission; {
        std::lock_guard<std::mutex> lock(mutex_mission_);
        mission = i_mission_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    ////////////////////// TODO //////////////////////
    // TODO: Add polyfit lane algorithm
    interface::PolyfitLanes poly_lanes;

    // TODO: Add find driving way algorithm
    interface::PolyfitLane  driving_way;

    // TODO: Add lateral and longitudinal control algorithm
    interface::VehicleCommand vehicle_command;

    //////////////////////////////////////////////////

    if (cfg_.use_manual_inputs == false) {
        // 0. Different to previous practice, the lane point data is not sorted by lanes.
        //    You have to divide the points by their lane.

        std::vector<geometry_msgs::msg::Point> filtered_points;

        // Add all points from lane_points.point to filtered_points
        for (const auto& point : lane_points.point) {
            filtered_points.push_back(point);
        }

        RCLCPP_INFO(this->get_logger(), "Number of filtered points: %zu", filtered_points.size());
        
        //sort by x in increasing order
        std::sort(filtered_points.begin(), filtered_points.end(), [](const auto& a, const auto& b) {
            return a.x < b.x;
        });

        // Threshold to consider a point as part of a new lane based on y-distance
        double lane_threshold = 2.0;

        std::vector<geometry_msgs::msg::Point> right_lane, left_lane;
        geometry_msgs::msg::Point last_left, last_right;

        for (const auto& point : filtered_points) {
            if (point.x > -1.0 && point.x < 2.0) {
                if (right_lane.empty() && point.y < 0) {
                    right_lane.push_back(point);
                    last_right = point;
                } else if (left_lane.empty() && point.y > 0) {
                    left_lane.push_back(point);
                    last_left = point;
                }
                // Break if we have both initial right and left points
                if (!right_lane.empty() && !left_lane.empty()) {
                    break;
                }
            }
        }

        for (const auto& point : filtered_points) {
            // Check if this point belongs to the current right lane
            if (std::abs(point.y - last_right.y) < lane_threshold) {
                right_lane.push_back(point);
                last_right = point; // Update the last right lane point
            } 
            // Check if this point belongs to the current left lane
            else if (std::abs(point.y - last_left.y) < lane_threshold) {
                left_lane.push_back(point);
                last_left = point; // Update the last left lane point
            }
        }

        RCLCPP_INFO(this->get_logger(), "Right lane points: %zu", right_lane.size());
        RCLCPP_INFO(this->get_logger(), "Left lane points: %zu", left_lane.size());

        // 1. With divided points, you can curve fit the lane and find the left, right lane.
        //    The generated left and right lane should be stored in the "poly_lanes".
        //    If you do so, the Display node will visualize the lanes.

        // Use Eigen for matrix calculation (pseudo inverse)

        Eigen::MatrixXd A(left_lane.size(), 4);  // For cubic fitting (a3, a2, a1, a0)

        Eigen::VectorXd b(left_lane.size());

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

        Eigen::VectorXd left_coeffs = A.completeOrthogonalDecomposition().solve(b);

        // Do the same for the right lane

        Eigen::MatrixXd A_right(right_lane.size(), 4);

        Eigen::VectorXd b_right(right_lane.size());

        for (size_t i = 0; i < right_lane.size(); ++i) {

            double x = right_lane[i].x;

            b_right(i) = right_lane[i].y;

            A_right(i, 0) = std::pow(x, 3);
            A_right(i, 1) = std::pow(x, 2);
            A_right(i, 2) = x;
            A_right(i, 3) = 1.0;

        }

        Eigen::VectorXd right_coeffs = A_right.completeOrthogonalDecomposition().solve(b_right);

        poly_lanes.frame_id = param_vehicle_namespace_ + "/body";

        // store within poly_lanes

        // Create PolyfitLaneData for left and right lanes
        ad_msgs::msg::PolyfitLaneData left_polyline, right_polyline;

        // Set IDs or any unique identifiers for each lane fit
        left_polyline.frame_id = param_vehicle_namespace_ + "/body";
        left_polyline.id = "1";
        right_polyline.frame_id = param_vehicle_namespace_ + "/body";
        right_polyline.id = "2";

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

        // 2. Generate the center line(=driving_way) which the vehicle will follow.
        //    The generated center line should be stored in the "driving_way".
        //    If you do so, the Display node will visualize the center line.

        driving_way.a3 = (left_coeffs(0) + right_coeffs(0)) / 2.0;
        driving_way.a2 = (left_coeffs(1) + right_coeffs(1)) / 2.0;
        driving_way.a1 = (left_coeffs(2) + right_coeffs(2)) / 2.0;
        driving_way.a0 = (left_coeffs(3) + right_coeffs(3)) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Driving_way - a3: %f, a2: %f, a1: %f, a0: %f", 
        driving_way.a3, driving_way.a2, driving_way.a1, driving_way.a0);

        // 3. Calculate the lateral command (steering angle [rad])
        //    You can tune your controller using the ros parameter.
        //    We provide the example of 'Pure Pursuit' parameters, so you can edit and use them.

        // Define a smoothing factor (0 < alpha < 1)
        double alpha = 0.5; // Adjust this value for desired smoothing effect

        // Compute the original e_
        double e_ = driving_way.a0 * std::pow(param_m_Lookahead_distance, 3)
                + driving_way.a1 * std::pow(param_m_Lookahead_distance, 2)
                + driving_way.a2 * param_m_Lookahead_distance
                + driving_way.a3;

        // stop if out of track
        if (filtered_points.size() == 0){
            vehicle_command.accel = 0.0;
            vehicle_command.brake = 1.0;
            e_                    = last_e_;        //maintain last steering
        }

        // Apply the low-pass filter
        e_ = alpha * e_ + (1.0 - alpha) * last_e_;
        last_e_ = e_;

        // Now use filtered_e_ for steering calculation
        double steering = atan((2 * param_pp_kd_ * e_) / (param_pp_kv_ * vehicle_state.velocity + param_pp_kc_));
        vehicle_command.steering = std::clamp(steering, -max_steering_angle, max_steering_angle);

        // 4. Calculate the longitudinal command(acceleration[0~1] or brake[0~1])
        //    You can tune your controller using the ros parameter.
        //    We provide the example of 'PID' parameters, so you can edit and use them.

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
