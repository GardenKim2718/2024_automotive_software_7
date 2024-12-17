#ifndef LANE_PROCESSING_NODE_HPP
#define LANE_PROCESSING_NODE_HPP

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "ad_msgs/msg/lane_point_data.hpp"
#include "ad_msgs/msg/polyfit_lane_data.hpp"
#include "ad_msgs/msg/polyfit_lane_data_array.hpp"

class LaneProcessingNode : public rclcpp::Node {
public:
    explicit LaneProcessingNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LaneProcessingNode() = default;

private:
    void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg);

    // Lane Detection and Clustering
    std::vector<std::vector<geometry_msgs::msg::Point>> dbscanClustering(
        const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight);

    // Lane Fitting and Filtering
    std::vector<double> generateSavitzkyGolayKernel(int window_size, int poly_order);
    std::vector<double> applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel);
    Eigen::VectorXd fitPolynomial(const std::vector<geometry_msgs::msg::Point>& points, int order);

    // Subscribers and Publishers
    rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr s_lane_points_;
    rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;

    // Parameters
    double eps_, x_weight_;
    int min_points_, window_size_, poly_order_;
    double lane_width_;
};

#endif // LANE_PROCESSING_NODE_HPP
