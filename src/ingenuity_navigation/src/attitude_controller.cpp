#include "ingenuity_navigation/attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

AttitudeController::AttitudeController(const Config &config) : config_(config)
{
    if (!std::isfinite(config.kp) || !std::isfinite(config.kd) ||
        !std::isfinite(config.max_swashplate_angle) ||
        config.kp < 0.0 || config.kd < 0.0 ||
        config.max_swashplate_angle <= 0.0 ||
        config.max_swashplate_angle > 0.15)
    {
        throw std::invalid_argument("Invalid attitude PD configuration");
    }
}

AttitudeController::Output AttitudeController::update(
    const Quaternion &target, const Quaternion &current,
    const Vector3 &body_angular_velocity) const
{
    if (!ingenuity::finite(body_angular_velocity))
    {
        return {};
    }

    const Quaternion target_unit = ingenuity::normalized(target);
    const Quaternion current_unit = ingenuity::normalized(current);

    // Erro de atitude no referencial do corpo: q_atual^-1 * q_alvo.
    Quaternion error = ingenuity::multiply(
        ingenuity::conjugate(current_unit), target_unit);

    // q e -q representam a mesma orientação. Escolher a menor rotação.
    if (error.w < 0.0)
    {
        error = {-error.x, -error.y, -error.z, -error.w};
    }

    const double vector_norm = std::sqrt(
        error.x * error.x + error.y * error.y + error.z * error.z);
    const double angle = 2.0 * std::atan2(vector_norm, error.w);
    const double scale = vector_norm > 1e-9 ? angle / vector_norm : 2.0;

    // A correção de yaw (componente Z) NÃO é utilizada nesta fase.
    const double roll_error = error.x * scale;
    const double pitch_error = error.y * scale;
    const double roll_u = config_.kp * roll_error -
                          config_.kd * body_angular_velocity.x;
    const double pitch_u = config_.kp * pitch_error -
                           config_.kd * body_angular_velocity.y;

    return {
        std::clamp(pitch_u, -config_.max_swashplate_angle,
                   config_.max_swashplate_angle),
        std::clamp(roll_u, -config_.max_swashplate_angle,
                   config_.max_swashplate_angle)
    };
}
