#ifndef PERSEVERANCE_NAVIGATION_HPP
#define PERSEVERANCE_NAVIGATION_HPP

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class PerseveranceNavigation : public rclcpp::Node
{
public:
    PerseveranceNavigation();

    void stopRobot();

private:

    // =========================================================
    // Configuration
    // =========================================================

    void loadParameters();
    void setupRosInterfaces();

    double target_x_ = 5.0;
    double target_y_ = 3.0;

    double kp_linear_ = 0.5;
    double kp_angular_ = 1.0;

    double max_linear_speed_ = 0.3;
    double max_angular_speed_ = 0.5;

    double distance_tolerance_ = 0.2;
    double angle_tolerance_ = 0.2;

    // =========================================================
    // Dynamic Parameters
    // =========================================================

    void setupParameterCallbacks();

    rcl_interfaces::msg::SetParametersResult validateParameters(
        const std::vector<rclcpp::Parameter> &parameters
    ) const;

    void updateParameters(
        const std::vector<rclcpp::Parameter> &parameters
    );


    // =========================================================
    // Robot State
    // =========================================================

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double distance_error_ = 0.0;
    double angle_error_ = 0.0;

    bool odometry_received_ = false;
    bool goal_reached_ = false;

    std::chrono::steady_clock::time_point last_odometry_time_;


    // =========================================================
    // ROS 2 Interfaces
    // =========================================================

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameter_validation_handle_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
        parameter_update_handle_;
    // =========================================================
    // Odometry
    // =========================================================

    void odometryCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg
    );

    bool odometryIsValid() const;


    // =========================================================
    // Navigation
    // =========================================================

    void calculateNavigationErrors();

    bool goalReached() const;


    // =========================================================
    // Control
    // =========================================================

    void controlLoop();

    geometry_msgs::msg::Twist calculateVelocityCommand() const;
};

#endif