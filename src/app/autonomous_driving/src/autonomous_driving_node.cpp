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
 * 2024-12-15 Chungwon Kim introduced DBSCAN algorithm and Savitzky-Golay filter within Lane Detection
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
    this->declare_parameter<double>("param_pp_kd", 1.0);
    this->declare_parameter<double>("param_pp_kv", 0.05);
    this->declare_parameter<double>("param_pp_kc", 0.5);
    this->declare_parameter<double>("param_pid_kp", 5.0);
    this->declare_parameter<double>("param_pid_ki", 0.002);
    this->declare_parameter<double>("param_pid_kd", 0.01);
    this->declare_parameter<int>("window_size", 7);
    this->declare_parameter<int>("poly_order", 2);
    this->declare_parameter<double>("eps", 5.0);
    this->declare_parameter<double>("x_weight", 0.05);
    this->declare_parameter<int>("min_points", 3);

    this->get_parameter("param_pp_kd", param_pp_kd_);
    this->get_parameter("param_pp_kv", param_pp_kv_);
    this->get_parameter("param_pp_kc", param_pp_kc_);
    this->get_parameter("param_pid_kp", param_pid_kp_);
    this->get_parameter("param_pid_ki", param_pid_ki_);
    this->get_parameter("param_pid_kd", param_pid_kd_);
    this->get_parameter("window_size", window_size);
    this->get_parameter("poly_order", poly_order);
    this->get_parameter("eps", eps);
    this->get_parameter("x_weight", x_weight);
    this->get_parameter("min_points", min_points);

    //////////////////////////////////////////////////
    ProcessParams();

    // Print the parameters
    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "use_manual_inputs: %d", cfg_.use_manual_inputs);
    RCLCPP_INFO(this->get_logger(), "eps: %f", eps);
    RCLCPP_INFO(this->get_logger(), "x_weight: %f", x_weight);
    RCLCPP_INFO(this->get_logger(), "min_points: %d", min_points);
    RCLCPP_INFO(this->get_logger(), "param_pp_kd: %f", param_pp_kd_);
    RCLCPP_INFO(this->get_logger(), "param_pp_kv: %f", param_pp_kv_);
    RCLCPP_INFO(this->get_logger(), "param_pp_kc: %f", param_pp_kc_);
    RCLCPP_INFO(this->get_logger(), "param_pid_kp: %f", param_pid_kp_);
    RCLCPP_INFO(this->get_logger(), "param_pid_ki: %f", param_pid_ki_);
    RCLCPP_INFO(this->get_logger(), "param_pid_kd: %f", param_pid_kd_);
    RCLCPP_INFO(this->get_logger(), "window_size: %d", window_size);
    RCLCPP_INFO(this->get_logger(), "poly_order: %d", poly_order);

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

// DBSCAN for lane point classification //
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

// SG filter for smoothing lane //
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

    ////////////// Lane Detection //////////////////
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
        if (point.x > -8.0 && point.x < 8.0) { // Valid x range
            if (point.y < 0) {
                potential_right.push_back(point); // Points likely on the right lane
            } else if (point.y > 0) {
                potential_left.push_back(point);  // Points likely on the left lane
            }
        }
    }

    // run DBSCAN for Lane Detection
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

    b_is_left_lane_empty = left_lane_points.empty();
    b_is_right_lane_empty = right_lane_points.empty();

    RCLCPP_INFO(this->get_logger(), "Right lane points: %zu", right_lane_points.size());
    RCLCPP_INFO(this->get_logger(), "Left lane points: %zu", left_lane_points.size());

    ////////////// Lane Fitting //////////////////
    //    With classified points, you can curve fit the lane and find the left, right lane.
    //    The generated left and right lane should be stored in the "poly_lanes".
    //    If you do so, the Display node will visualize the lanes.

    // Perform polynomial fitting for left lane
    Eigen::MatrixXd A_left(left_lane_points.size(), 3);  // A_left : Matrix for fitting the cubic polynomial for the left lane
    Eigen::VectorXd B_left(left_lane_points.size()); // b_left : Vector for the y-coordinates of the left lane points
    Eigen::MatrixXd A_right(right_lane_points.size(), 3); // A_right : Matrix for fitting the cubic polynomial for the right lane
    Eigen::VectorXd B_right(right_lane_points.size()); // b_right : Vector for the y-coordinates of the right lane points
    Eigen::VectorXd left_coeffs, right_coeffs;
    interface::PolyfitLane left_polyline, right_polyline;
    
    left_polyline.frame_id = cfg_.vehicle_namespace + "/body"; // left_polyline.frame_id : Frame of reference for left lane polyline
    left_polyline.id = "1"; // left_polyline.id : Unique identifier for the left lane
    right_polyline.frame_id = cfg_.vehicle_namespace + "/body"; // right_polyline.frame_id : Frame of reference for right lane polyline
    right_polyline.id = "2"; // right_polyline.id : Unique identifier for the right lane

    // if lane points are not empty, execute polynomial(quadratic) fitting
    if (!b_is_left_lane_empty) {
        for (size_t i = 0; i < left_lane_points.size(); ++i) {
            double x = left_lane_points[i].x;
            B_left(i) = left_lane_points[i].y;
            A_left(i, 0) = 1.0;
            A_left(i, 1) = x;
            A_left(i, 2) = std::pow(x, 2);
        }
        left_coeffs = A_left.completeOrthogonalDecomposition().solve(B_left); // left_coeffs : Coefficients for the left lane polynomial
        left_polyline.a0 = left_coeffs(0);
        left_polyline.a1 = left_coeffs(1);
        left_polyline.a2 = left_coeffs(2);
        left_polyline.a3 = 0.0;
    }

    if (!b_is_right_lane_empty) {
        for (size_t i = 0; i < right_lane_points.size(); ++i) {
            double x = right_lane_points[i].x;
            B_right(i) = right_lane_points[i].y;
            A_right(i, 0) = 1.0;
            A_right(i, 1) = x;
            A_right(i, 2) = std::pow(x, 2);
        }
        right_coeffs = A_right.completeOrthogonalDecomposition().solve(B_right); // right_coeffs : Coefficients for the right lane polynomial
        right_polyline.a0 = right_coeffs(0);
        right_polyline.a1 = right_coeffs(1);
        right_polyline.a2 = right_coeffs(2);
        right_polyline.a3 = 0.0;
    }

    // if one lane is empty, fill it with the other
    if (b_is_left_lane_empty && !b_is_right_lane_empty) {
        left_polyline.a0 = right_coeffs(0) + lane_width;
        left_polyline.a1 = right_coeffs(1);
        left_polyline.a2 = right_coeffs(2);
        left_polyline.a3 = 0.0;
    }

    if (!b_is_left_lane_empty && b_is_right_lane_empty) {
        right_polyline.a0 = left_coeffs(0) - lane_width;
        right_polyline.a1 = left_coeffs(1);
        right_polyline.a2 = left_coeffs(2);
        right_polyline.a3 = 0.0;
    }
    // if both lanes are empty just maintain course
    if (b_is_left_lane_empty && b_is_right_lane_empty) {
        RCLCPP_INFO(this->get_logger(), "NO Lane Detected!!!");
        left_polyline.a0 = lane_width/2;
        left_polyline.a1 = 0.0;
        left_polyline.a2 = 0.0;
        left_polyline.a3 = 0.0;
        right_polyline.a0 = -lane_width/2;
        right_polyline.a1 = 0.0;
        right_polyline.a2 = 0.0;
        right_polyline.a3 = 0.0;
    }

    // Verify both lanes show similar coefficients
    if (!b_is_left_lane_empty && !b_is_right_lane_empty) {
        const double fit_tolerance = 1.5; // Define a tolerance threshold for slope comparison
        double left_slope = 2 * left_coeffs(2) * param_m_Lookahead_distance + left_coeffs(1);  // Slope = 2*a2*x + a1 for quadratic polynomial
        double right_slope = 2 * right_coeffs(2) * param_m_Lookahead_distance + right_coeffs(1);  // Slope = 2*a2*x + a1 for quadratic polynomial
        double slope_diff = left_slope - right_slope; // Compute the difference between the slopes of the left and right lanes

        // Check if the slope difference is greater than the tolerance threshold
        if (std::abs(slope_diff) > fit_tolerance) {
            if (std::abs(left_slope) > std::abs(right_slope)) { // If the left slope is much steeper (greater absolute value) than the right slope
                // Adjust the left lane's coefficients to match the right lane's coefficients
                left_polyline.a1 = right_polyline.a1;  // Copy the slope of the right lane to the left lane
                left_polyline.a2 = right_polyline.a2;  // Copy the curvature of the right lane to the left lane
            }
            else { // If the right slope is much steeper (greater absolute value) than the left slope
                // Adjust the right lane's coefficients to match the left lane's coefficients
                right_polyline.a1 = left_polyline.a1;  // Copy the slope of the left lane to the right lane
                right_polyline.a2 = left_polyline.a2;  // Copy the curvature of the left lane to the right lane
            }
        }
    }

    // Add the polylines to the poly_lanes' polyfitlanes list
    poly_lanes.polyfitlanes.push_back(left_polyline);
    poly_lanes.polyfitlanes.push_back(right_polyline);

    // Generate center driving lane as the average of left and right lanes
    driving_way.a3 = (left_polyline.a3 + right_polyline.a3) / 2.0;
    driving_way.a2 = (left_polyline.a2 + right_polyline.a2) / 2.0;
    driving_way.a1 = (left_polyline.a1 + right_polyline.a1) / 2.0;
    driving_way.a0 = (left_polyline.a0 + right_polyline.a0) / 2.0;
    // RCLCPP_INFO(this->get_logger(), "Driving_way - a3: %f, a2: %f, a1: %f, a0: %f", driving_way.a3, driving_way.a2, driving_way.a1, driving_way.a0);

    ////////////// Path Planning //////////////////

    // Initialize boolean flags
    b_trigger_merge = false;                // b_trigger_merge : trigger for lane merge
    b_is_left_lane_empty = true;           // b_is_left_lane_empty : left lane is empty
    b_is_right_lane_empty = true;          // b_is_right_lane_empty : right lane is empty
    b_is_icy_road = false;
    b_is_up_slope = false;
    b_is_down_slope = false;
    b_left_merge = true;
    b_right_merge = true;
    b_is_merge_safe = true;

    // Extract ego vehicle's position
    double ego_x = current_vehicle_state.x;
    double ego_y = current_vehicle_state.y;
    double ego_yaw = current_vehicle_state.yaw;

    // Vectors to store static and dynamic obstacles
    std::vector<ObstacleInfo> static_obstacles;
    std::vector<ObstacleInfo> dynamic_obstacles;

    for (const auto& obstacle : current_mission.objects) {
        // Calculate the distance to the obstacle
        double delta_x = obstacle.x - ego_x;
        double delta_y = obstacle.y - ego_y;

        // Convert obstacle coordinates to ego coordinates
        double cos_yaw = std::cos(ego_yaw);
        double sin_yaw = std::sin(ego_yaw);
        double obs_x_ego = cos_yaw * delta_x + sin_yaw * delta_y;
        double obs_y_ego = -sin_yaw * delta_x + cos_yaw * delta_y;

        // Create an ObstacleInfo object
        ObstacleInfo obs_info = {obs_x_ego, obs_y_ego, obstacle.velocity};
        
        // Classify the obstacle and update minimum distances
        if (obstacle.object_type == "Static") {
            static_obstacles.push_back(obs_info);
        } else if (obstacle.object_type == "Dynamic") {
            dynamic_obstacles.push_back(obs_info);
        }
    }

    // Set road condition and slope flags
    b_is_icy_road = (current_mission.road_condition == "Ice");
    b_is_up_slope = (current_mission.road_slope == "Up");
    b_is_down_slope = (current_mission.road_slope == "Down");

    double min_dynamic_distance = 100.0;
    double min_static_distance = 100.0;
    double obs_velocity = 35.0;
    const double lane_width_threshold = (lane_width / 2.0) + 1.0; // Define a threshold for same lane detection

    // Detect obstacles in the same lane
    for (const auto& obstacle : static_obstacles) {
        double obs_distance = std::sqrt(std::pow(obstacle.x_ego, 2) + std::pow(obstacle.y_ego, 2));
        if (obstacle.x_ego > 0 && std::abs(obstacle.y_ego) < lane_width_threshold) {  // Check if the obstacle is ahead and within the lane width
            b_trigger_merge = true; // Trigger merge if there is a static obstacle ahead
            if (obs_distance < min_static_distance) {
                min_static_distance = obs_distance;
            }
        }
    }

    // Check which lane is safe to merge
    if (b_trigger_merge) {
        RCLCPP_INFO(this->get_logger(), "Merge Triggered!!!");

        for (const auto& obstacle : static_obstacles) {
            if (obstacle.x_ego > 0) {  // Check if the obstacle is ahead
                if (obstacle.y_ego > lane_width_threshold && obstacle.y_ego < 3 * lane_width_threshold) {  // Check if the obstacle is in the left lane
                    b_left_merge = false;
                } else if (obstacle.y_ego < -lane_width_threshold && obstacle.y_ego > -3 * lane_width_threshold) {  // Check if the obstacle is in the right lane
                    b_right_merge = false;
                }
            }
        }
    }

    // Classify forward dynamic obstacles
    for (const auto& obstacle : dynamic_obstacles) {
        double obs_distance = std::sqrt(std::pow(obstacle.x_ego, 2) + std::pow(obstacle.y_ego, 2));
        if (obstacle.x_ego > 0 && std::abs(obstacle.y_ego) < lane_width_threshold) {  // Check if the obstacle is ahead and within the lane width
            if (obs_distance < min_dynamic_distance) {
                min_dynamic_distance = obs_distance;
                obs_velocity = obstacle.velocity;
            }
        }
    }

    ////////////// Vehicle Control //////////////////

    // Lateral control: Calculate steering angle based on Pure Pursuit //
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
    double min_distance = 100.0;

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

    if (b_is_icy_road) { // If the road is icy, reduce the target speed
        target_speed = std::min(target_speed, icy_speed);
        RCLCPP_INFO(this->get_logger(), "Icy Road Detected!!!");
    }

    if (steering > steering_threshold || steering < -steering_threshold) {
        target_speed = min_speed;
        RCLCPP_INFO(this->get_logger(), "Steering Angle Exceeds Threshold");
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

    // If using manual input
    if (cfg_.use_manual_inputs == true) {
        vehicle_command = manual_input;
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