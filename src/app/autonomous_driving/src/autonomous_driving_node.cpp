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

double weightedEuclideanDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b, double x_weight) {
    double dx = (a.x - b.x) * x_weight; // Weight the x-dimension
    double dy = (a.y - b.y);            // Keep y-dimension as is
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::vector<geometry_msgs::msg::Point>> AutonomousDriving::dbscanClustering(
    const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight) {

    std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
    std::vector<bool> visited(points.size(), false);
    std::vector<bool> noise(points.size(), false);

    // Neighborhood query with weighted distance
    auto regionQuery = [&](size_t index) {
        std::vector<size_t> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            if (i != index && weightedEuclideanDistance(points[index], points[i], x_weight) <= eps) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    };

    // Expand cluster
    auto expandCluster = [&](size_t index, std::vector<size_t> neighbors, std::vector<bool>& visited) {
        std::vector<geometry_msgs::msg::Point> cluster;
        cluster.push_back(points[index]);
        visited[index] = true;

        for (size_t i = 0; i < neighbors.size(); ++i) {
            size_t neighbor_index = neighbors[i];
            if (!visited[neighbor_index]) {
                visited[neighbor_index] = true;
                auto sub_neighbors = regionQuery(neighbor_index);
                if (sub_neighbors.size() >= static_cast<size_t>(min_points)) {
                    neighbors.insert(neighbors.end(), sub_neighbors.begin(), sub_neighbors.end());
                }
            }
            if (!noise[neighbor_index]) {
                cluster.push_back(points[neighbor_index]);
                noise[neighbor_index] = true;
            }
        }
        clusters.push_back(cluster);
    };

    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;

        auto neighbors = regionQuery(i);
        if (neighbors.size() < static_cast<size_t>(min_points)) {
            noise[i] = true;
        } else {
            expandCluster(i, neighbors, visited);
        }
    }
    return clusters;
}

std::vector<double> AutonomousDriving::generateSavitzkyGolayKernel(int window_size, int poly_order) {
    int half_window = window_size / 2;
    Eigen::MatrixXd A(window_size, poly_order + 1);
    // Fill the Vandermonde matrix
    for (int i = -half_window; i <= half_window; ++i) {
        for (int j = 0; j <= poly_order; ++j) {
            A(i + half_window, j) = std::pow(i, j);
        }
    }
    // Compute the pseudoinverse
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::MatrixXd At = A.transpose();
    Eigen::MatrixXd pinv = AtA.ldlt().solve(At);
    // Extract the middle row for the convolution kernel
    std::vector<double> kernel(window_size);
    for (int i = 0; i < window_size; ++i) {
        kernel[i] = pinv(0, i); // Use the 0th row for smoothing
    }
    return kernel;
}
// Apply Savitzky-Golay filter to a data series
std::vector<double> AutonomousDriving::applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel) {
    int half_window = kernel.size() / 2;
    std::vector<double> smoothed_data(data.size(), 0.0);
    // Convolution with kernel
    for (size_t i = 0; i < data.size(); ++i) {
        double smoothed_value = 0.0;
        for (int j = -half_window; j <= half_window; ++j) {
            int index = std::clamp(static_cast<int>(i) + j, 0, static_cast<int>(data.size() - 1));
            smoothed_value += data[index] * kernel[j + half_window];
        }
        smoothed_data[i] = smoothed_value;
    }
    return smoothed_data;
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
        filtered_lane_points.push_back(converted_point); // Add to filtered_lane_points
    }

    RCLCPP_INFO(this->get_logger(), "Number of filtered points: %zu", filtered_lane_points.size());

    // Sort points based on x-coordinates
    std::sort(filtered_lane_points.begin(), filtered_lane_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });

    std::vector<geometry_msgs::msg::Point> potential_left, potential_right;

    for (const auto& point : filtered_lane_points) {
        if (point.x > -5.0 && point.x < 5.0) { // Valid x range
            if (point.y < 0) {
                potential_right.push_back(point); // Points likely on the right lane
            } else if (point.y > 0) {
                potential_left.push_back(point);  // Points likely on the left lane
            }
        }
    }

    // DBSCAN algorithm
    double eps = 5.0;      // Maximum distance for a point to be considered part of a cluster
    double x_weight = 0.1; // Weight for x-dimension
    int min_points = 3;    // Minimum number of points to form a cluster

    auto left_clusters = dbscanClustering(potential_left, eps, min_points, x_weight);
    auto right_clusters = dbscanClustering(potential_right, eps, min_points, x_weight);

    std::vector<geometry_msgs::msg::Point> left_lane_points, right_lane_points;

    if (!left_clusters.empty()) {
        // Find the largest cluster in left_clusters
        auto max_left_cluster = *std::max_element(
            left_clusters.begin(), left_clusters.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); });
        
        // Flatten the largest cluster into left_lane_points
        left_lane_points.insert(left_lane_points.end(), max_left_cluster.begin(), max_left_cluster.end());
    }

    if (!right_clusters.empty()) {
        // Find the largest cluster in right_clusters
        auto max_right_cluster = *std::max_element(
            right_clusters.begin(), right_clusters.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); });
        
        // Flatten the largest cluster into right_lane_points
        right_lane_points.insert(right_lane_points.end(), max_right_cluster.begin(), max_right_cluster.end());
    }

    // Savitzky-Golay filter
    std::vector<double> left_x, left_y, right_x, right_y;
    for (const auto& point : left_lane_points) {
        left_x.push_back(point.x);
        left_y.push_back(point.y);
    }
    for (const auto& point : right_lane_points) {
        right_x.push_back(point.x);
        right_y.push_back(point.y);
    }

    // Generate Savitzky-Golay kernel
    int window_size = 7;  // Odd number
    int poly_order = 2;   // Cubic smoothing
    auto kernel = generateSavitzkyGolayKernel(window_size, poly_order);

    // Smooth lane points
    std::vector<double> smoothed_left_x = applySavitzkyGolayFilter(left_x, kernel);
    std::vector<double> smoothed_left_y = applySavitzkyGolayFilter(left_y, kernel);
    std::vector<double> smoothed_right_x = applySavitzkyGolayFilter(right_x, kernel);
    std::vector<double> smoothed_right_y = applySavitzkyGolayFilter(right_y, kernel);
    // Replace original points with smoothed points
    left_lane_points.clear();
    right_lane_points.clear();
    for (size_t i = 0; i < smoothed_left_x.size(); ++i) {
        geometry_msgs::msg::Point point;
        point.x = smoothed_left_x[i];
        point.y = smoothed_left_y[i];
        left_lane_points.push_back(point);
    }
    for (size_t i = 0; i < smoothed_right_x.size(); ++i) {
        geometry_msgs::msg::Point point;
        point.x = smoothed_right_x[i];
        point.y = smoothed_right_y[i];
        right_lane_points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Right lane points: %zu", right_lane_points.size());
    RCLCPP_INFO(this->get_logger(), "Left lane points: %zu", left_lane_points.size());

    /* >>>>>>> Lane Fitting <<<<<<<*/
    //    With classified points, you can curve fit the lane and find the left, right lane.
    //    The generated left and right lane should be stored in the "poly_lanes".
    //    If you do so, the Display node will visualize the lanes.

    // Perform polynomial fitting for left lane
    Eigen::MatrixXd A_left(left_lane_points.size(), 3);  // A_left : Matrix for fitting the cubic polynomial for the left lane
    Eigen::VectorXd b_left(left_lane_points.size()); // b_left : Vector for the y-coordinates of the left lane points
    Eigen::MatrixXd A_right(right_lane_points.size(), 3); // A_right : Matrix for fitting the cubic polynomial for the right lane
    Eigen::VectorXd b_right(right_lane_points.size()); // b_right : Vector for the y-coordinates of the right lane points

    // Fill the matrix A and vector b with left lane points
    for (size_t i = 0; i < left_lane_points.size(); ++i) {
        double x = left_lane_points[i].x;
        b_left(i) = left_lane_points[i].y;
        A_left(i, 0) = 1.0;
        A_left(i, 1) = x;
        A_left(i, 2) = std::pow(x, 2);
    }

    // Fill the matrix A and vector b with right lane points
    for (size_t i = 0; i < right_lane_points.size(); ++i) {
        double x = right_lane_points[i].x;
        b_right(i) = right_lane_points[i].y;
        A_right(i, 0) = 1.0;
        A_right(i, 1) = x;
        A_right(i, 2) = std::pow(x, 2);
    }

    // Polynomial fitting (quadratic)
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
    left_polyline.a3 = 0.0;

    right_polyline.a0 = right_coeffs(0);
    right_polyline.a1 = right_coeffs(1);
    right_polyline.a2 = right_coeffs(2);
    right_polyline.a3 = 0.0;

    // Add the polylines to the poly_lanes' polyfitlanes list
    poly_lanes.polyfitlanes.push_back(left_polyline);
    poly_lanes.polyfitlanes.push_back(right_polyline);

    // Generate center driving lane as the average of left and right lanes
    //    The generated center line should be stored in the "driving_way".
    //    If you do so, the Display node will visualize the center line.
    driving_way.a3 = (left_coeffs(3) + right_coeffs(3)) / 2.0;
    driving_way.a2 = (left_coeffs(2) + right_coeffs(2)) / 2.0;
    driving_way.a1 = (left_coeffs(1) + right_coeffs(1)) / 2.0;
    driving_way.a0 = 0.0; //(left_coeffs(0) + right_coeffs(0)) / 2.0;
    
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
        lateral_error = driving_way.a3 * std::pow(param_m_Lookahead_distance, 3)
                + driving_way.a2 * std::pow(param_m_Lookahead_distance, 2)
                + driving_way.a1 * param_m_Lookahead_distance
                + driving_way.a0;

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