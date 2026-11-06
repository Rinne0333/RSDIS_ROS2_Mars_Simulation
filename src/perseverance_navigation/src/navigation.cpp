#include "perseverance_navigation/navigation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;


// =============================================================
// Constructor
// =============================================================

PerseveranceNavigation::PerseveranceNavigation() : Node("perseverance_navigation")
{
    loadParameters();
    setupRosInterfaces();

    RCLCPP_INFO(
        get_logger(),
        "Navigation initialized | Target: (%.2f, %.2f)",
        target_x_,
        target_y_
    );
}
    
// =============================================================
// Configuration
// =============================================================

void PerseveranceNavigation::loadParameters()
{
    target_x_ = declare_parameter<double>("target_x", target_x_);
    target_y_ = declare_parameter<double>("target_y", target_y_);

    kp_linear_ = declare_parameter<double>("kp_linear", kp_linear_);
    kp_angular_ = declare_parameter<double>("kp_angular", kp_angular_);

    max_linear_speed_ = declare_parameter<double>(
        "max_linear_speed", max_linear_speed_
    );

    max_angular_speed_ = declare_parameter<double>(
        "max_angular_speed", max_angular_speed_
    );

    distance_tolerance_ = declare_parameter<double>(
        "distance_tolerance", distance_tolerance_
    );

    angle_tolerance_ = declare_parameter<double>(
        "angle_tolerance", angle_tolerance_
    );

    const double values[] = {
        target_x_, target_y_,
        kp_linear_, kp_angular_,
        max_linear_speed_, max_angular_speed_,
        distance_tolerance_, angle_tolerance_
    };

    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            throw std::invalid_argument(
                "Navigation parameters must be finite"
            );
        }
    }

    if (kp_linear_ <= 0.0 ||
        kp_angular_ <= 0.0 ||
        max_linear_speed_ <= 0.0 ||
        max_angular_speed_ <= 0.0 ||
        distance_tolerance_ <= 0.0 ||
        angle_tolerance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Controller gains, limits and tolerances must be positive"
        );
    }
}

// =============================================================
// ROS 2 Interfaces
// =============================================================

void PerseveranceNavigation::setupRosInterfaces()
{
    // Odometry subscriber
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/perseverance/odometry",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            odometryCallback(msg);
        }
    );

    // Velocity publisher
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/perseverance/cmd_vel",
        10
    );

    // Control loop: 10 Hz
    control_timer_ = create_wall_timer(
        100ms,
        [this]()
        {
            controlLoop();
        }
    );
}

// =============================================================
// Odometry
// =============================================================

void PerseveranceNavigation::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;

    // Current position
    const double new_x = position.x;
    const double new_y = position.y;

    // Quaternion to yaw
    const double siny_cosp = 2.0 * (
        orientation.w * orientation.z +
        orientation.x * orientation.y
    );

    const double cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y +
        orientation.z * orientation.z
    );

    const double new_yaw = std::atan2(siny_cosp, cosy_cosp);

    if (!std::isfinite(new_x) ||
        !std::isfinite(new_y) ||
        !std::isfinite(new_yaw))
    {
        RCLCPP_WARN(
            get_logger(),
            "Invalid odometry received"
        );

        odometry_received_ = false;
        return;
    }

    x_ = new_x;
    y_ = new_y;
    yaw_ = new_yaw;

    last_odometry_time_ = std::chrono::steady_clock::now();
    odometry_received_ = true;
}


bool PerseveranceNavigation::odometryIsValid() const
{
    if (!odometry_received_)
    {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();

    const double age = std::chrono::duration<double>(
        now - last_odometry_time_
    ).count();

    return age <= 0.5;
}

// =============================================================
// Navigation
// =============================================================

void PerseveranceNavigation::calculateNavigationErrors()
{
    const double dx = target_x_ - x_;
    const double dy = target_y_ - y_;

    // Distance to destination
    distance_error_ = std::hypot(dx, dy);

    // Desired orientation
    const double target_yaw = std::atan2(dy, dx);

    // Normalized angular error
    angle_error_ = std::atan2(
        std::sin(target_yaw - yaw_),
        std::cos(target_yaw - yaw_)
    );
}


bool PerseveranceNavigation::goalReached() const
{
    return distance_error_ <= distance_tolerance_;
}

// =============================================================
// Proportional Controller
// =============================================================

geometry_msgs::msg::Twist
PerseveranceNavigation::calculateVelocityCommand() const
{
    geometry_msgs::msg::Twist cmd;

    // Angular controller
    cmd.angular.z = std::clamp(
        kp_angular_ * angle_error_,
        -max_angular_speed_,
        max_angular_speed_
    );

    // Linear controller
    if (std::abs(angle_error_) <= angle_tolerance_)
    {
        cmd.linear.x = std::clamp(
            kp_linear_ * distance_error_,
            0.0,
            max_linear_speed_
        );
    }

    return cmd;
}

// =============================================================
// Control Loop
// =============================================================

void PerseveranceNavigation::controlLoop()
{
    // Safety: stop if odometry is unavailable
    if (!odometryIsValid())
    {
        stopRobot();
        return;
    }

    // Navigation already completed
    if (goal_reached_)
    {
        stopRobot();
        return;
    }

    // Update navigation errors
    calculateNavigationErrors();

    // Check destination
    if (goalReached())
    {
        goal_reached_ = true;
        stopRobot();

        RCLCPP_INFO(
            get_logger(),
            "Destination reached!"
        );

        return;
    }

    // Calculate and publish velocity
    const auto cmd = calculateVelocityCommand();

    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Distance: %.2f m | Angle: %.2f rad | V: %.2f | W: %.2f",
        distance_error_,
        angle_error_,
        cmd.linear.x,
        cmd.angular.z
    );
}


// =============================================================
// Stop
// =============================================================

void PerseveranceNavigation::stopRobot()
{
    if (!cmd_pub_)
    {
        return;
    }

    cmd_pub_->publish(geometry_msgs::msg::Twist{});
}