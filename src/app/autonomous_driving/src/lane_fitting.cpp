#include "lane_processing_node.hpp"

LaneProcessingNode::LaneProcessingNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {

    // Declare parameters
    this->declare_parameter<double>("eps", 5.0);
    this->declare_parameter<double>("x_weight", 0.05);
    this->declare_parameter<int>("min_points", 3);
    this->declare_parameter<int>("window_size", 7);
    this->declare_parameter<int>("poly_order", 2);
    this->declare_parameter<double>("lane_width", 3.0);

    this->get_parameter("eps", eps_);
    this->get_parameter("x_weight", x_weight_);
    this->get_parameter("min_points", min_points_);
    this->get_parameter("window_size", window_size_);
    this->get_parameter("poly_order", poly_order_);
    this->get_parameter("lane_width", lane_width_);

    // Subscriber
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", 10, std::bind(&LaneProcessingNode::CallbackLanePoints, this, std::placeholders::_1));

    // Publisher
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>("poly_lanes", 10);
}

void LaneProcessingNode::CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
    // Lane Point Clustering
    std::vector<geometry_msgs::msg::Point> potential_left, potential_right;

    for (const auto& point : msg->point) {
        if (point.x > -8.0 && point.x < 8.0) {
            if (point.y < 0) potential_right.push_back(point);
            else if (point.y > 0) potential_left.push_back(point);
        }
    }

    auto left_clusters = dbscanClustering(potential_left, eps_, min_points_, x_weight_);
    auto right_clusters = dbscanClustering(potential_right, eps_, min_points_, x_weight_);

    std::vector<geometry_msgs::msg::Point> left_lane, right_lane;

    if (!left_clusters.empty()) {
        auto max_left = *std::max_element(left_clusters.begin(), left_clusters.end(),
                                          [](auto& a, auto& b) { return a.size() < b.size(); });
        left_lane = max_left;
    }

    if (!right_clusters.empty()) {
        auto max_right = *std::max_element(right_clusters.begin(), right_clusters.end(),
                                           [](auto& a, auto& b) { return a.size() < b.size(); });
        right_lane = max_right;
    }

    // Lane Fitting
    std::vector<double> left_x, left_y, right_x, right_y;

    for (const auto& point : left_lane) { left_x.push_back(point.x); left_y.push_back(point.y); }
    for (const auto& point : right_lane) { right_x.push_back(point.x); right_y.push_back(point.y); }

    auto kernel = generateSavitzkyGolayKernel(window_size_, poly_order_);
    auto smoothed_left_x = applySavitzkyGolayFilter(left_x, kernel);
    auto smoothed_left_y = applySavitzkyGolayFilter(left_y, kernel);
    auto smoothed_right_x = applySavitzkyGolayFilter(right_x, kernel);
    auto smoothed_right_y = applySavitzkyGolayFilter(right_y, kernel);

    // Prepare output
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes_msg;

    // Left lane fitting
    auto left_coeffs = fitPolynomial(left_lane, 2);
    ad_msgs::msg::PolyfitLaneData left_lane_msg;
    left_lane_msg.a0 = left_coeffs(0);
    left_lane_msg.a1 = left_coeffs(1);
    left_lane_msg.a2 = left_coeffs(2);

    // Right lane fitting
    auto right_coeffs = fitPolynomial(right_lane, 2);
    ad_msgs::msg::PolyfitLaneData right_lane_msg;
    right_lane_msg.a0 = right_coeffs(0);
    right_lane_msg.a1 = right_coeffs(1);
    right_lane_msg.a2 = right_coeffs(2);

    // Publish
    poly_lanes_msg.polyfitlanes.push_back(left_lane_msg);
    poly_lanes_msg.polyfitlanes.push_back(right_lane_msg);
    p_poly_lanes_->publish(poly_lanes_msg);
}

// DBSCAN, Savitzky-Golay, Polynomial fitting helper functions implementation
