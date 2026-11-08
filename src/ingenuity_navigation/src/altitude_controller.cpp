#include "ingenuity_navigation/altitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

AltitudeController::AltitudeController(const Config &config) : config_(config)
{
    if (!std::isfinite(config.kp) || !std::isfinite(config.ki) ||
        !std::isfinite(config.kd) || !std::isfinite(config.hover_thrust) ||
        !std::isfinite(config.min_thrust) || !std::isfinite(config.max_thrust) ||
        !std::isfinite(config.integral_limit) ||
        config.kp < 0.0 || config.ki < 0.0 || config.kd < 0.0 ||
        config.hover_thrust <= 0.0 || config.min_thrust < 0.0 ||
        config.max_thrust <= config.min_thrust || config.integral_limit < 0.0)
    {
        throw std::invalid_argument("Invalid altitude PID configuration");
    }
}

double AltitudeController::update(double target_z, double current_z,
                                  double world_velocity_z, double dt)
{
    if (!std::isfinite(target_z) || !std::isfinite(current_z) ||
        !std::isfinite(world_velocity_z) || !std::isfinite(dt) || dt <= 0.0)
    {
        // Nunca propagar NaN para os atuadores.
        return 0.0;
    }

    const double error = target_z - current_z;

    // Derivada aplicada à medição, para evitar 'derivative kick' no setpoint.
    const double proportional_derivative =
        config_.hover_thrust + config_.kp * error -
        config_.kd * world_velocity_z;

    const double candidate_integral = std::clamp(
        integral_error_ + error * dt,
        -config_.integral_limit, config_.integral_limit);

    const double candidate_output =
        proportional_derivative + config_.ki * candidate_integral;

    // Anti-windup condicional: não acumular integral se a saturação
    // impedir que o controlador corrija o erro no sentido necessário.
    const bool saturated_high = candidate_output > config_.max_thrust;
    const bool saturated_low = candidate_output < config_.min_thrust;
    if ((!saturated_high || error < 0.0) &&
        (!saturated_low || error > 0.0))
    {
        integral_error_ = candidate_integral;
    }

    return std::clamp(
        proportional_derivative + config_.ki * integral_error_,
        config_.min_thrust, config_.max_thrust);
}

void AltitudeController::reset()
{
    integral_error_ = 0.0;
}
