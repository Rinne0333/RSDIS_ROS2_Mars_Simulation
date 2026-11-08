/**
 * @brief Class PerseveranceNavigation declaration
 * @details Here is defined the public and private elements of the PerseveranceNavigation Class
 */
#ifndef PERSEVERANCE_NAVIGATION_HPP
#define PERSEVERANCE_NAVIGATION_HPP

#include <chrono>   // Time management
#include <memory>   // Optimizes memory management e.g auto node = ...
#include <vector>   // Usage of vectors
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class PerseveranceNavigation : public rclcpp::Node
{
public:
    // Public FUnctions called in the Node
    PerseveranceNavigation(void);
    void stopRobot(void);

private:
    // Configuration
    void loadParameters(void);
    void setupRosInterfaces(void);
    
    double target_x_ = 5.0;      
    double target_y_ = 3.0;

    double kp_linear_ = 0.5;
    double kp_angular_ = 1.0;

    double max_linear_speed_ = 1.0;
    double max_angular_speed_ = 1.0;

    double distance_tolerance_ = 0.2;
    double angle_tolerance_ = 0.2;


    // Dynamic Parameters
    void setupParameterCallbacks(void);
    rcl_interfaces::msg::SetParametersResult validateParameters(const std::vector<rclcpp::Parameter> &parameters) const;
    void updateParameters(const std::vector<rclcpp::Parameter> &parameters);

    // Robot State
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double distance_error_ = 0.0;
    double angle_error_ = 0.0;

    bool odometry_received_ = false;
    bool goal_reached_ = false;

    std::chrono::steady_clock::time_point last_odometry_time_;

    // Obstacle detection
    double stop_distance_ = 1.0;                // Frontal obstacle stop distance
    double detection_half_width_ = 0.45;        // Half width of analysed region
    double detection_min_y_ = -0.30;            // Superior vertical limit of optical refereencial
    double detection_max_y_ = 0.30;             // Inferior vertical limit of optical refereencial

    // Nº obst points in each region
    int obstacle_points_left_ = 0;
    int obstacle_points_front_ = 0;
    int obstacle_points_right_ = 0;

    int min_obstacle_points_ = 5;               // Minimum proximity points to confirm an obstacle

    // Obstacle presence in each region.
    bool obstacle_left_ = false;
    bool obstacle_front_ = false;
    bool obstacle_right_ = false;

    bool obstacle_detected_ = false;

    bool pointcloud_received_ = false;

    std::chrono::steady_clock::time_point last_pointcloud_time_;

    // ROS 2 Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;   
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validation_handle_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_update_handle_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Odometry
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool odometryIsValid(void) const;

    // Obstacle Detection
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    // Sttechart with Perseverance Navigation States
    enum class NavigationMode
    {
        IDLE = 0,
        GO_TO_GOAL,
        FOLLOW_OBSTACLE
    };

    NavigationMode navigation_mode_ = NavigationMode::IDLE;

    // Navigation
    void calculateNavigationErrors(void);
    bool goalReached(void) const;

    // Control
    geometry_msgs::msg::Twist calculateVelocityCommand(void) const;
    void controlLoop(void);
};

#endif