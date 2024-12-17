#ifndef LANE_FITTING_HPP
#define LANE_FITTING_HPP

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>

class LaneFitting {
public:
    LaneFitting(int window_size, int poly_order);

    std::vector<double> generateSavitzkyGolayKernel();
    std::vector<double> applySavitzkyGolayFilter(const std::vector<double>& data, const std::vector<double>& kernel);

    std::vector<std::vector<geometry_msgs::msg::Point>> dbscanClustering(
        const std::vector<geometry_msgs::msg::Point>& points, double eps, int min_points, double x_weight);

    Eigen::VectorXd fitPolynomial(const std::vector<geometry_msgs::msg::Point>& points, int order);

private:
    int window_size_;
    int poly_order_;
};

#endif // LANE_FITTING_HPP
