#include "lane_processing_node.hpp"

LaneProcessingNode::LaneProcessingNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {

    // Declare Parameters
    this->declare_parameter("eps", 5.0);
    this->declare_parameter("x_weight", 0.05);
    this->declare_parameter("min_points", 3);
    this->declare_parameter("window_size", 7);
    this->declare_parameter("poly_order", 2);
    this->declare_parameter("lane_width", 3.5);

    this->get_parameter("eps", eps_);
    this->get_parameter("x_weight", x_weight_);
    this->get_parameter("min_points", min_points_);
    this->get_parameter("window_size", window_size_);
    this->get_parameter("poly_order", poly_order_);
    this->get_parameter("lane_width", lane_width_);

    // Subscriber
    s_sensor_data_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", 10, std::bind(&LaneProcessingNode::CallbackSensorData, this, std::placeholders::_1));

    // Publisher
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>("poly_lanes", 10);

    RCLCPP_INFO(this->get_logger(), "LaneProcessingNode initialized.");
}

void LaneProcessingNode::CallbackSensorData(const ad_msgs::msg::LanePointData::SharedPtr msg) {
    std::vector<geometry_msgs::msg::Point> filtered_points = msg->point;
    std::vector<geometry_msgs::msg::Point> left_lane, right_lane;

    // Classify Points
    for (const auto& point : filtered_points) {
        if (point.x > -8.0 && point.x < 8.0) {
            if (point.y > 0) {
                left_lane.push_back(point);
            } else if (point.y < 0) {
                right_lane.push_back(point);
            }
        }
    }

    // Perform DBSCAN Clustering
    auto left_clusters = dbscanClustering(left_lane, eps_, min_points_, x_weight_);
    auto right_clusters = dbscanClustering(right_lane, eps_, min_points_, x_weight_);

    auto getLargestCluster = [](const auto& clusters) {
        if (!clusters.empty()) {
            return *std::max_element(clusters.begin(), clusters.end(), [](const auto& a, const auto& b) {
                return a.size() < b.size();
            });
        }
        return std::vector<geometry_msgs::msg::Point>{};
    };

    auto kernel = generateSavitzkyGolayKernel(window_size_, poly_order_);

    auto smoothPoints = [&](const std::vector<geometry_msgs::msg::Point>& points) {
        std::vector<double> x, y;
        for (const auto& point : points) {
            x.push_back(point.x);
            y.push_back(point.y);
        }
        auto smoothed_x = applySavitzkyGolayFilter(x, kernel);
        auto smoothed_y = applySavitzkyGolayFilter(y, kernel);

        std::vector<geometry_msgs::msg::Point> smoothed_points;
        for (size_t i = 0; i < smoothed_x.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = smoothed_x[i];
            p.y = smoothed_y[i];
            smoothed_points.push_back(p);
        }
        return smoothed_points;
    };

    auto largest_left = smoothPoints(getLargestCluster(left_clusters));
    auto largest_right = smoothPoints(getLargestCluster(right_clusters));

    // Polynomial Fitting
    auto left_coeffs = fitPolynomial(largest_left, 2);
    auto right_coeffs = fitPolynomial(largest_right, 2);
    Eigen::VectorXd driving_way_coeffs = (left_coeffs + right_coeffs) / 2.0;

    // Publish PolyfitLaneDataArray
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes_msg;
    poly_lanes_msg.frame_id = "lane_frame";

    ad_msgs::msg::PolyfitLaneData left_lane_data, right_lane_data, driving_way_data;

    left_lane_data.id = "1";
    left_lane_data.a0 = left_coeffs(0);
    left_lane_data.a1 = left_coeffs(1);
    left_lane_data.a2 = left_coeffs(2);

    right_lane_data.id = "2";
    right_lane_data.a0 = right_coeffs(0);
    right_lane_data.a1 = right_coeffs(1);
    right_lane_data.a2 = right_coeffs(2);

    driving_way_data.id = "center";
    driving_way_data.a0 = driving_way_coeffs(0);
    driving_way_data.a1 = driving_way_coeffs(1);
    driving_way_data.a2 = driving_way_coeffs(2);

    poly_lanes_msg.polyfitlanes.push_back(left_lane_data);
    poly_lanes_msg.polyfitlanes.push_back(right_lane_data);
    poly_lanes_msg.polyfitlanes.push_back(driving_way_data);

    p_poly_lanes_->publish(poly_lanes_msg);
    RCLCPP_INFO(this->get_logger(), "Published left, right, and center driving lanes.");
}

std::vector<std::vector<geometry_msgs::msg::Point>> LaneProcessingNode::dbscanClustering(
    const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight) {
    // Implement DBSCAN logic here...
    return {};
}

std::vector<double> LaneProcessingNode::generateSavitzkyGolayKernel(int window_size, int poly_order) {
    int half_window = window_size / 2;
    Eigen::MatrixXd A(window_size, poly_order + 1);

    for (int i = -half_window; i <= half_window; ++i) {
        for (int j = 0; j <= poly_order; ++j) {
            A(i + half_window, j) = std::pow(i, j);
        }
    }
    Eigen::MatrixXd pinv = (A.transpose() * A).ldlt().solve(A.transpose());
    std::vector<double> kernel(window_size);

    for (int i = 0; i < window_size; ++i) {
        kernel[i] = pinv(0, i);
    }
    return kernel;
}

std::vector<double> LaneProcessingNode::applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel) {
    int half_window = kernel.size() / 2;
    std::vector<double> smoothed_data(data.size(), 0.0);

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

Eigen::VectorXd LaneProcessingNode::fitPolynomial(const std::vector<geometry_msgs::msg::Point>& points, int order) {
    int n = points.size();
    Eigen::MatrixXd A(n, order + 1);
    Eigen::VectorXd B(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].x;
        B(i) = points[i].y;
        for (int j = 0; j <= order; ++j) {
            A(i, j) = std::pow(x, j);
        }
    }
    return A.colPivHouseholderQr().solve(B);
}
