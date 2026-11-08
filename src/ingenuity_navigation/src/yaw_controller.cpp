#include "ingenuity_navigation/yaw_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

YawController::YawController(const Config &config) : config_(config)
{
    if (!std::isfinite(config.kp) || !std::isfinite(config.kd) ||
        !std::isfinite(config.max_diff_n) || config.kp < 0.0 ||
        config.kd < 0.0 || config.max_diff_n <= 0.0 ||
        config.max_diff_n > 10.0)
    {
        throw std::invalid_argument("Invalid yaw controller configuration");
    }
}

double YawController::wrapAngle(double angle)
{
    // Evita descontinuidades em +/-pi.
    return std::atan2(std::sin(angle), std::cos(angle));
}

double YawController::update(double target_heading_world,
                             double current_heading_world,
                             double yaw_rate_world) const
{
    if (!std::isfinite(target_heading_world) ||
        !std::isfinite(current_heading_world) ||
        !std::isfinite(yaw_rate_world))
    {
        throw std::invalid_argument("Invalid yaw feedback");
    }

    const double error = wrapAngle(target_heading_world - current_heading_world);
    const double diff = config_.kp * error - config_.kd * yaw_rate_world;
    return std::clamp(diff, -config_.max_diff_n, config_.max_diff_n);
}
