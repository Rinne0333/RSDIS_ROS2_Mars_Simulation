#include "perseverance_navigation/navigation.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;

/**
 * @brief PerseveranceNavigation Class Constructor
 * @details Called when creating a new object from Preseverance Class
 */
PerseveranceNavigation::PerseveranceNavigation(void) : Node("perseverance_navigation")
{
    loadParameters();
    setupParameterCallbacks();
    setupRosInterfaces();

    // Similar to a printf: get_logger() shows the Node who is sending the message
    RCLCPP_INFO(
        get_logger(),   
        "Navigation initialized | Target: %.2f, %.2f",
        target_x_,
        target_y_
    );
}
    
/**
 * @brief Loads Parameters in the Constructor
 * @details Declares Parameters using declare_parameter from Node class
 */
void PerseveranceNavigation::loadParameters(void)
{
    target_x_ = declare_parameter<double>(
        "target_x", target_x_
    );
    
    target_y_ = declare_parameter<double>(
        "target_y", target_y_
    );

    kp_linear_ = declare_parameter<double>(
        "kp_linear", kp_linear_
    );
    
    kp_angular_ = declare_parameter<double>(
        "kp_angular", kp_angular_
    );

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

    stop_distance_ = declare_parameter<double>(
        "stop_distance", stop_distance_
    );

    detection_half_width_ = declare_parameter<double>(
        "detection_half_width", detection_half_width_
    );

    detection_min_y_ = declare_parameter<double>(
        "detection_min_y", detection_min_y_
    );

    detection_max_y_ = declare_parameter<double>(
        "detection_max_y", detection_max_y_
    );

    min_obstacle_points_ = declare_parameter<int>(
        "min_obstacle_points", min_obstacle_points_
    );
}

/**
 * @brief Sets Parameters Callbacks when setted in CLI
 * @details Validates the Value and then Updates it
 */
void PerseveranceNavigation::setupParameterCallbacks(void)
{
    // Validate parameters before accepting changes
    parameter_validation_handle_ =
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters)
            {
                return validateParameters(parameters);
            }
        );

    // Update internal variables after successful changes
    parameter_update_handle_ =
        this->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters)
            {
                updateParameters(parameters);
            }
        );
}

/**
 * @brief Verifies if new parameter value is valid before ROS2 accepting it
 * @details SetParametersResult: bool successful + std::string reason
 */
rcl_interfaces::msg::SetParametersResult PerseveranceNavigation::validateParameters(
    const std::vector<rclcpp::Parameter> &parameters) const
{
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = true;

    // Reads the entire array  to check if the name of the parameter corresponds to an existing one
    for (const auto &param : parameters)
    {
        const std::string name = param.get_name();

        const bool is_controller_parameter =
            name == "kp_linear" ||
            name == "kp_angular" ||
            name == "max_linear_speed" ||
            name == "max_angular_speed" ||
            name == "distance_tolerance" ||
            name == "angle_tolerance";

        
        if (!is_controller_parameter)
        {
            continue;
        }

        // All controller parameters must be doubles
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            result.successful = false;
            result.reason = "Parameter value must be a double";
            return result;
        }

        const double value = param.as_double();

        // Reject invalid controller values
        if (!std::isfinite(value) || value <= 0.0)
        {
            result.successful = false;
            result.reason = name + " must be finite and positive";
            return result;
        }
    }

    return result;
}

/**
 * @brief Updates Parameter Value when Validation result succeeds
 */
void PerseveranceNavigation::updateParameters(const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &param : parameters)
    {
        const std::string name = param.get_name();

        if (name == "kp_linear")
        {
            kp_linear_ = param.as_double();
        }
        else if (name == "kp_angular")
        {
            kp_angular_ = param.as_double();
        }
        else if (name == "max_linear_speed")
        {
            max_linear_speed_ = param.as_double();
        }
        else if (name == "max_angular_speed")
        {
            max_angular_speed_ = param.as_double();
        }
        else if (name == "distance_tolerance")
        {
            distance_tolerance_ = param.as_double();
        }
        else if (name == "angle_tolerance")
        {
            angle_tolerance_ = param.as_double();
        }
        else
        {
            continue;
        }

        RCLCPP_INFO(
            get_logger(),
            "Parameter updated: %s = %.2f",
            name.c_str(),
            param.as_double()
        );
    }
}

/**
 * @brief Sets ROS2 Interfaces of the PerseveranceNavigation Node
 * @details PerseveranceNavigation Node needs Odometry, Speed Command, control loop
 */
void PerseveranceNavigation::setupRosInterfaces(void)
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

    // Depth camera subscriber
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perseverance/depth_camera/points",
        rclcpp::SensorDataQoS(),
        [this] (const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
        {
            pointcloudCallback(msg);
        }
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

/**
 * @brief Odometry Callback  
 * @details Reads Odometry message and Updates Robot's current position and orientation
 */
void PerseveranceNavigation::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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


bool PerseveranceNavigation::odometryIsValid(void) const
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

/**
 * @brief Obstacle Detection callback
 */
void PerseveranceNavigation::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
)
{
    int obstacle_points = 0;
    int valid_points = 0;

    try
    {
        // Traverse X, Y, Z coordinates
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (;
             iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z)
        {
            const double x = *iter_x;
            const double y = *iter_y;
            const double z = *iter_z;

            // Ignore invalid measurements
            if (!std::isfinite(x) ||
                !std::isfinite(y) ||
                !std::isfinite(z))
            {
                continue;
            }

            ++valid_points;

            // Check frontal detection region
            const bool inside_region =
                std::abs(x) <= detection_half_width_ &&
                y >= detection_min_y_ &&
                y <= detection_max_y_ &&
                z > 0.0 &&
                z < stop_distance_;

            if (inside_region)
            {
                ++obstacle_points;
            }
        }
    }

    catch (const std::runtime_error &ex)
    {
        pointcloud_received_ = false;
        obstacle_detected_ = true;

        RCLCPP_WARN(
            get_logger(),
            "Invalid PointCloud2: %s",
            ex.what()
        );

        return;
    }

    // Reject empty or unusable point clouds
    if (valid_points == 0)
    {
        pointcloud_received_ = false;
        obstacle_detected_ = true;
        return;
    }

    last_pointcloud_time_ = std::chrono::steady_clock::now();

    pointcloud_received_ = true;

    const bool new_obstacle_state = obstacle_points >= min_obstacle_points_;

    // Log only when obstacle state changes
    if (new_obstacle_state != obstacle_detected_)
    {
        if (new_obstacle_state)
        {
            RCLCPP_WARN(
                get_logger(),
                "OBSTACLE DETECTED: %d points",
                obstacle_points
            );
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "Obstacle cleared"
            );
        }
    }

    obstacle_detected_ = new_obstacle_state;
}

// Navigation
void PerseveranceNavigation::calculateNavigationErrors(void)
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


bool PerseveranceNavigation::goalReached(void) const
{
    return distance_error_ <= distance_tolerance_;
}

// Proportional Controller
geometry_msgs::msg::Twist PerseveranceNavigation::calculateVelocityCommand(void) const
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

// Control Loop
void PerseveranceNavigation::controlLoop(void)
{
    // Safety: stop if odometry is unavailable
    if (!odometryIsValid())
    {
        stopRobot();
        return;
    }

    // Check depth camera availability
    if (!pointcloud_received_)
    {
        stopRobot();


        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Waiting for valid depth camera data"
        );

        return;
    }

    // Check depth camera timeout
    const auto now = std::chrono::steady_clock::now();

    const double cloud_age =
        std::chrono::duration<double>(
            now - last_pointcloud_time_
        ).count();

    if (cloud_age > 0.5)
    {
        stopRobot();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Depth camera timeout"
        );

        return;
    }

    // Stop if an obstacle is detected
    if (obstacle_detected_)
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

// Stop Perseverance
void PerseveranceNavigation::stopRobot(void)
{
    if (!cmd_pub_)
    {
        return;
    }

    cmd_pub_->publish(geometry_msgs::msg::Twist{});
}