/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      display_node.cpp
 * @brief     display topics using markers
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-10-12 updated by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *              : added dynamic obstacle vehicle
 *            2024-11-05 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : clean up
 */

#include "display/display_node.hpp"

Display::Display(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
        
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameter init      
    this->declare_parameter("display/ns", "");
    this->declare_parameter("display/loop_rate_hz", 30.0);
    ProcessParams();
    
    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    
    std::string dir(getenv("PWD"));
    std::string mesh_path("/resources/meshes");
    cfg_.mesh_dir = dir + mesh_path;
    RCLCPP_INFO(this->get_logger(), "mesh_path: %s", cfg_.mesh_dir.c_str());

    // Subscriber init
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState> (
        "vehicle_state", qos_profile, std::bind(&Display::CallbackVehicleState, this, std::placeholders::_1));
    s_mission_ = this->create_subscription<ad_msgs::msg::MissionDisplay>(
        "mission_display", qos_profile, std::bind(&Display::CallbackMission, this, std::placeholders::_1));
    s_csv_lanes_ = this->create_subscription<ad_msgs::msg::LanePointDataArray> (
        "csv_lanes", qos_profile, std::bind(&Display::CallbackCsvLanes, this, std::placeholders::_1));
    s_roi_lanes_ = this->create_subscription<ad_msgs::msg::LanePointDataArray> (
        "ROI_lanes", qos_profile, std::bind(&Display::CallbackROILanes, this, std::placeholders::_1));
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData> (
        "lane_points", qos_profile, std::bind(&Display::CallbackLanePoints, this, std::placeholders::_1));
    s_poly_lanes_ = this->create_subscription<ad_msgs::msg::PolyfitLaneDataArray> (
        "poly_lanes", qos_profile, std::bind(&Display::CallbackPolyLanes, this, std::placeholders::_1));
    s_driving_way_ = this->create_subscription<ad_msgs::msg::PolyfitLaneData> (
        "driving_way", qos_profile, std::bind(&Display::CallbackDrivingWay, this, std::placeholders::_1));
    
    // Publisher init
    p_vehicle_marker_ = this->create_publisher<visualization_msgs::msg::Marker> (
        "vehicle_marker", qos_profile);
    p_ego_vehicle_velocity_ = this->create_publisher<std_msgs::msg::Float32> (
        "/ego_vehicle_velocity", qos_profile);
    p_mission_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "mission_marker", qos_profile);
    p_csv_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "csv_lanes_marker", qos_profile);
    p_lane_points_marker_ = this->create_publisher<visualization_msgs::msg::Marker> (
        "lane_points_marker", qos_profile);
    p_roi_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "ROI_lanes_marker", qos_profile);
    p_poly_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "polyfit_lanes_marker", qos_profile);
    p_driving_way_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "driving_way_marker", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); }); 

    RCLCPP_WARN_STREAM(this->get_logger(), "Initialize node (Period: " << cfg_.loop_rate_hz << " Hz)"); //cfg_.loop_rate_hz
}

Display::~Display() {}

void Display::ProcessParams() {
    this->get_parameter("display/ns", cfg_.vehicle_namespace);
    this->get_parameter("display/loop_rate_hz", cfg_.loop_rate_hz);
}

void Display::Run() {
    auto current_time = this->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    ad_msgs::msg::VehicleState vehicle_state; {
        if (b_is_vehicle_state_ == true) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            vehicle_state = i_vehicle_state_;
        }
    }

    ad_msgs::msg::LanePointDataArray csv_lanes; {
        if (b_is_csv_lanes_ == true) {
            std::lock_guard<std::mutex> lock(mutex_csv_lanes_);
            csv_lanes = i_csv_lanes_;
        }
    }

    ad_msgs::msg::LanePointDataArray roi_lanes; {
        if (b_is_roi_lanes_ == true) {
            std::lock_guard<std::mutex> lock(mutex_roi_lanes_);
            roi_lanes = i_roi_lanes_;
        }
    }

    ad_msgs::msg::LanePointData lane_points; {
        if (b_is_lane_points_ == true) {
            std::lock_guard<std::mutex> lock(mutex_lane_points_);
            lane_points = i_lane_points_;
        }
    }

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes; {
        if (b_is_poly_lanes_ == true) {
            std::lock_guard<std::mutex> lock(mutex_poly_lanes_);
            poly_lanes = i_poly_lanes_;
        }
    }

    ad_msgs::msg::PolyfitLaneData driving_way; {
        if (b_is_driving_way_ == true) {
            std::lock_guard<std::mutex> lock(mutex_driving_way_);
            driving_way = i_driving_way_;
        }
    }

    ad_msgs::msg::MissionDisplay mission; {
        if (b_is_mission_ == true) {
            std::lock_guard<std::mutex> lock(mutex_mission_);
            mission = i_mission_;
        }
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - // 
    if (b_is_vehicle_state_ == true) {
        DisplayVehicle(vehicle_state, current_time, cfg_);

        if (b_is_mission_ == true) {
            DisplayMission(mission, vehicle_state, current_time, cfg_);
        }
    }

    if (b_is_csv_lanes_ == true) {
        if ((current_time.seconds() - time_csv_lanes_marker_) > 1.0) {
            time_csv_lanes_marker_ = current_time.seconds();
            DisplayCsvLanes(csv_lanes, current_time);
        }
    }

    if (b_is_roi_lanes_ == true) {
        if ((current_time.seconds() - time_roi_lanes_marker_) > 0.2) {
            time_roi_lanes_marker_ = current_time.seconds();
            DisplayROILanes(roi_lanes, current_time);
        }
    }

    if (b_is_lane_points_ == true) {
        if ((current_time.seconds() - time_lane_points_marker_) > 0.2) {
            time_lane_points_marker_ = current_time.seconds();
            DisplayLanePoints(lane_points, current_time);
        }
    }

    if (b_is_poly_lanes_ == true) {
        if ((current_time.seconds() - time_poly_lanes_marker_) > 0.2) {
            time_poly_lanes_marker_ = current_time.seconds();
            DisplayPolyLanes(poly_lanes, current_time, 0.1, 30.0);
        }
    }

    if (b_is_driving_way_ == true) {
        if ((current_time.seconds() - time_driving_way_marker_) > 0.2) {
            time_driving_way_marker_ = current_time.seconds();
            DisplayDrivingWay(driving_way, current_time, 0.1, 30.0);
        }
    }
}

void Display::DisplayVehicle(const ad_msgs::msg::VehicleState& vehicle_state,
                             const rclcpp::Time& current_time,
                             const DisplayConfig& cfg) {

    tf2::Quaternion q_temp;
    tf2::Matrix3x3 m(q_temp);
    q_temp.setRPY(0.0 / 180.0 * M_PI, 0, 180.0 / 180.0 * M_PI);
    tf2::Quaternion q(q_temp.getX(), q_temp.getY(), q_temp.getZ(), q_temp.getW());

    visualization_msgs::msg::Marker vehicle_marker;
    vehicle_marker.header.frame_id = cfg.vehicle_namespace + "/body";
    vehicle_marker.header.stamp = current_time;
    vehicle_marker.ns = vehicle_state.id;
    vehicle_marker.id = 0;
    vehicle_marker.lifetime = rclcpp::Duration(0, int64_t(0.1*1e9));

    // Set the marker type
    vehicle_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    vehicle_marker.mesh_resource = "file://" + cfg.mesh_dir + "/Ioniq5.stl";
    vehicle_marker.mesh_use_embedded_materials = true;

    vehicle_marker.pose.position.x = 1.5;
    vehicle_marker.pose.position.y = 0.0;
    vehicle_marker.pose.position.z = 0.0;
    vehicle_marker.pose.orientation.x = q.getX();
    vehicle_marker.pose.orientation.y = q.getY();
    vehicle_marker.pose.orientation.z = q.getZ();
    vehicle_marker.pose.orientation.w = q.getW();

    // Set the scale of the marker
    vehicle_marker.scale.x = 1.0;
    vehicle_marker.scale.y = 1.0;
    vehicle_marker.scale.z = 1.0;

    vehicle_marker.color.r = 1.0;
    vehicle_marker.color.g = 1.0;
    vehicle_marker.color.b = 1.0;
    vehicle_marker.color.a = 1.0;

    p_vehicle_marker_->publish(vehicle_marker);

    std_msgs::msg::Float32 ego_vehicle_velocity;
    ego_vehicle_velocity.data = vehicle_state.velocity;

    p_ego_vehicle_velocity_->publish(ego_vehicle_velocity);
}

void Display::DisplayMission(const ad_msgs::msg::MissionDisplay& mission,
                             const ad_msgs::msg::VehicleState& vehicle_state,
                             const rclcpp::Time& current_time,
                             const DisplayConfig& cfg) {

    visualization_msgs::msg::MarkerArray mission_marker_array;
    int id = 0;

    for (auto obj : mission.objects) {
        visualization_msgs::msg::Marker obj_marker;
        obj_marker.header.frame_id = "world";
        obj_marker.header.stamp = current_time;
        obj_marker.ns = "object";
        obj_marker.id = id;

        // Set the marker type
        obj_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        if(obj.object_type.compare("Static") == 0) {
            obj_marker.mesh_resource = "file://" + cfg.mesh_dir + "/barrier.stl";
        }
        else if (obj.object_type.compare("Dynamic") == 0) {
            obj_marker.mesh_resource = "file://" + cfg.mesh_dir + "/Ioniq5.stl";
        }
        obj_marker.action = visualization_msgs::msg::Marker::ADD;
        obj_marker.mesh_use_embedded_materials = true;

        obj_marker.pose.position.x = obj.x;
        obj_marker.pose.position.y = obj.y;
        obj_marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, obj.yaw + M_PI);
        q.normalize();
        obj_marker.pose.orientation.x = q.x();
        obj_marker.pose.orientation.y = q.y();
        obj_marker.pose.orientation.z = q.z();
        obj_marker.pose.orientation.w = q.w();

        // Set the scale of the marker
        obj_marker.scale.x = 1.0;
        obj_marker.scale.y = 1.0;
        obj_marker.scale.z = 1.0;

        // Color
        if(obj.object_type.compare("Static") == 0) {
            obj_marker.color.r = 1.0f;
            obj_marker.color.g = 0.753f;
            obj_marker.color.b = 0.796f;
            obj_marker.color.a = 0.3f;
        }
        else if (obj.object_type.compare("Dynamic") == 0) {
            obj_marker.color.r = 0.0f;
            obj_marker.color.g = 1.0f;
            obj_marker.color.b = 0.0f;
            obj_marker.color.a = 0.3f;
        }

        // Color changer based on distance
        double dx = vehicle_state.x - obj.x;
        double dy = vehicle_state.y - obj.y;
        double det_len = sqrt(dx*dx + dy*dy);
        
        if(det_len < 15){
            obj_marker.color.a = 1.0;
        } 

        obj_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        mission_marker_array.markers.push_back(obj_marker);

        id++;
    }

    for (auto region : mission.regions) {
        visualization_msgs::msg::Marker region_marker;
        region_marker.header.frame_id = "world";
        region_marker.header.stamp = current_time;
        region_marker.ns = "region";
        region_marker.id = id;

        // Set the marker type
        region_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        region_marker.action = visualization_msgs::msg::Marker::ADD;

        region_marker.pose.position.x = region.x;
        region_marker.pose.position.y = region.y;
        region_marker.pose.position.z = -0.05;

        region_marker.scale.x = 2.0 * region.radius;
        region_marker.scale.y = 2.0 * region.radius;
        region_marker.scale.z = 0.05;

        if(region.mission.compare("road_slope") == 0) {
            if(region.sub_type.compare("Up") == 0) {
                //  Uphill color : Red
                region_marker.color.r = 0.6f;
                region_marker.color.g = 0.0f;
                region_marker.color.b = 0.0f;
                region_marker.color.a = 0.7;
                
                mission_marker_array.markers.push_back(region_marker);
            }
            else if(region.sub_type.compare("Down") == 0) {
                //  Downhill color : Blue
                region_marker.color.r = 0.0f;
                region_marker.color.g = 0.0f;
                region_marker.color.b = 0.6f;
                region_marker.color.a = 0.7;

                mission_marker_array.markers.push_back(region_marker);
            }
        }
        else if (region.mission.compare("road_condition") == 0) {
            if (region.sub_type.compare("Ice") == 0) {
                //  Ice color : Skyblue
                region_marker.color.r = 0.684f;
                region_marker.color.g = 0.866f;
                region_marker.color.b = 0.940f;
                region_marker.color.a = 0.7;

                mission_marker_array.markers.push_back(region_marker);
            }
        }
        else if (region.mission.compare("speed_limit") == 0) {
            //  Speed limit color : Yellow
            region_marker.color.r = 0.6f;
            region_marker.color.g = 0.6f;
            region_marker.color.b = 0.0f;
            region_marker.color.a = 0.7;

            mission_marker_array.markers.push_back(region_marker);
        }

        id++;
    }
    
    p_mission_marker_->publish(mission_marker_array);
}

void Display::DisplayROILanes(const ad_msgs::msg::LanePointDataArray& roi_lanes,
                              const rclcpp::Time& current_time) {
    visualization_msgs::msg::MarkerArray markerArray;
    int id = 0;

    for (auto& lane : roi_lanes.lane) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = lane.frame_id;
        marker.header.stamp = current_time;

        marker.ns = lane.id;
        marker.id = id++;

        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.25;
        marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

        geometry_msgs::msg::Point prevPoint;
        bool first = true;

        for (auto& point : lane.point) {
            geometry_msgs::msg::Point currPoint;
            currPoint.x = point.x;
            currPoint.y = point.y;
            currPoint.z = 0.0;

            if (first == true) {
                first = false;
            } 
            else {
                double dx = currPoint.x - prevPoint.x;
                double dy = currPoint.y - prevPoint.y;
                if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
                    marker.points.push_back(prevPoint);
                    marker.points.push_back(currPoint);
                } 
                else {
                    markerArray.markers.push_back(marker);
                    marker.points.clear();
                    marker.id = id++;
                }
            }
            prevPoint = currPoint;
        }
        markerArray.markers.push_back(marker);
    }
    p_roi_lanes_marker_->publish(markerArray);
}

void Display::DisplayLanePoints(const ad_msgs::msg::LanePointData& lane_points,
                                const rclcpp::Time& current_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = lane_points.frame_id;
    marker.header.stamp = current_time;

    marker.ns = lane_points.id;
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

    for (auto& point : lane_points.point) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;

        marker.points.push_back(p);
    }

    p_lane_points_marker_->publish(marker);
}

void Display::DisplayPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray& poly_lanes,
                               const rclcpp::Time& current_time,
                               const double& interval, const double& ROILength) {

    visualization_msgs::msg::MarkerArray markerArray;

    for (auto& lane : poly_lanes.polyfitlanes) {
        double x = 0.0;
        double y = lane.a0;

        double distance_square = x * x + y * y;
        int id = 0;

        while (distance_square < ROILength * ROILength) {
            double a0 = lane.a0;
            double a1 = lane.a1;
            double a2 = lane.a2;
            double a3 = lane.a3;

            y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
            distance_square = x * x + y * y;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = lane.frame_id;
            marker.header.stamp = current_time;

            marker.ns = lane.id;
            marker.id = id++;

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.1;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

            markerArray.markers.push_back(marker);
            x += interval;
        }
    }
    p_poly_lanes_marker_->publish(markerArray);
}

void Display::DisplayDrivingWay(const ad_msgs::msg::PolyfitLaneData& driving_way,
                                const rclcpp::Time& current_time,
                                const double& interval, const double& ROILength) {

    double a0 = driving_way.a0;
    double a1 = driving_way.a1;
    double a2 = driving_way.a2;
    double a3 = driving_way.a3;

    double x = 0.0;
    double y = a0;

    double distance_square = x * x + y * y;
    int id = 0;

    visualization_msgs::msg::MarkerArray markerArray;
    while (distance_square < ROILength * ROILength) {

        y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
        distance_square = x * x + y * y;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = driving_way.frame_id;
        marker.header.stamp = current_time;

        marker.ns = driving_way.id;
        marker.id = id++;

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

        markerArray.markers.push_back(marker);
        x += interval;
    }
    p_driving_way_marker_->publish(markerArray);
}

void Display::DisplayCsvLanes(const ad_msgs::msg::LanePointDataArray& csv_lanes,
                              const rclcpp::Time& current_time) {

    visualization_msgs::msg::MarkerArray markerArray;

    int id = 0;
    double marker_rgb[] = {0.9f,0.9f,0.9f,0.7f};

    for (auto& lane : csv_lanes.lane) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = lane.frame_id;
        marker.header.stamp = current_time;
        marker.ns = lane.id;
        marker.id = id++;
        
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.color.r = marker_rgb[0];
        marker.color.g = marker_rgb[1];
        marker.color.b = marker_rgb[2];
        marker.color.a = 0.7;
        marker.scale.x = 0.2;
        marker.lifetime = rclcpp::Duration(0, 0);

        geometry_msgs::msg::Point prevPoint;
        bool first = true;

        for (auto& point : lane.point) {
            geometry_msgs::msg::Point currPoint;
            currPoint.x = point.x;
            currPoint.y = point.y;
            currPoint.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0.9f;
            color.g = 0.9f;
            color.b = 0.9f;
            color.a = 0.7;

            if (first == true) {
                first = false;
            } 
            else {
                double dx = currPoint.x - prevPoint.x;
                double dy = currPoint.y - prevPoint.y;
                if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
                    marker.points.push_back(prevPoint);
                    marker.points.push_back(currPoint);
                    marker.colors.push_back(color);
                    marker.colors.push_back(color);
                } 
                else {
                    markerArray.markers.push_back(marker);
                    marker.points.clear();
                    marker.colors.clear();
                    marker.id = id++;
                }
            }
            prevPoint = currPoint;
        }
        markerArray.markers.push_back(marker);
    }
    p_csv_lanes_marker_->publish(markerArray);
}

int main(int argc, char **argv) {
    std::string node_name = "display";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Display>(node_name));
    rclcpp::shutdown();

    return 0;
}
