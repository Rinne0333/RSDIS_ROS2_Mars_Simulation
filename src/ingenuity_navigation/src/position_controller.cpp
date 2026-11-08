#include "ingenuity_navigation/position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PositionController::PositionController(const Config &config) : config_(config)
{
    if (!std::isfinite(config.kp) || !std::isfinite(config.kd) ||
        !std::isfinite(config.max_tilt_rad) || !std::isfinite(config.gravity) ||
        config.kp < 0.0 || config.kd < 0.0 ||
        config.max_tilt_rad <= 0.0 || config.max_tilt_rad > 0.25 ||
        config.gravity <= 0.0 ||
        (config.thrust_axis_z != 1.0 && config.thrust_axis_z != -1.0))
    {
        throw std::invalid_argument("Invalid XY position PD configuration");
    }
}

PositionController::Output PositionController::update(
    const ingenuity::Vector3 &target_world,
    const ingenuity::Vector3 &current_world,
    const ingenuity::Vector3 &velocity_world,
    const ingenuity::Quaternion &hover_orientation,
    double gain_scale) const
{
    if (!ingenuity::finite(target_world) || !ingenuity::finite(current_world) ||
        !ingenuity::finite(velocity_world))
    {
        throw std::invalid_argument("Invalid position/velocity");
    }

    if (!std::isfinite(gain_scale) || gain_scale < 0.0 || gain_scale > 1.0)
    {
        throw std::invalid_argument("Invalid position gain scale");
    }

    // PD de posição em XY. O controlo vertical é de outra classe.
    ingenuity::Vector3 desired_acceleration{
        gain_scale * (config_.kp * (target_world.x - current_world.x) -
            config_.kd * velocity_world.x),
        gain_scale * (config_.kp * (target_world.y - current_world.y) -
            config_.kd * velocity_world.y),
        0.0
    };

    const double magnitude = ingenuity::horizontalNorm(desired_acceleration);
    const double max_acceleration =
        config_.gravity * std::tan(config_.max_tilt_rad);
    if (magnitude > max_acceleration)
    {
        desired_acceleration = desired_acceleration *
            (max_acceleration / magnitude);
    }

    // O vetor (ax, ay, g) é a direção desejada do empuxo no mundo.
    // Converte-se primeiro para o referencial de atitude de hover.
    ingenuity::Vector3 thrust_direction_world{
        desired_acceleration.x,
        desired_acceleration.y,
        config_.gravity
    };
    const double thrust_length = ingenuity::norm(thrust_direction_world);
    thrust_direction_world = thrust_direction_world * (1.0 / thrust_length);

    const auto q_hover = ingenuity::normalized(hover_orientation);
    const auto desired_local = ingenuity::rotate(
        ingenuity::conjugate(q_hover), thrust_direction_world);

    // Obtemos os ângulos que levam (0, 0, thrust_axis_z) à direção desejada.
    // Para o modelo atual thrust_axis_z=-1: roll positivo gera +Y local,
    // e pitch positivo gera -X local. Não assumimos corpo vertical +Z.
    const double s = config_.thrust_axis_z;
    const double roll = std::clamp(
        std::atan2(-s * desired_local.y, s * desired_local.z),
        -config_.max_tilt_rad, config_.max_tilt_rad);
    const double pitch = std::clamp(
        std::asin(std::clamp(s * desired_local.x, -1.0, 1.0)),
        -config_.max_tilt_rad, config_.max_tilt_rad);

    const auto local_tilt = ingenuity::rollPitchQuaternion(roll, pitch);
    const auto desired_attitude =
        ingenuity::normalized(ingenuity::multiply(q_hover, local_tilt));

    return {desired_attitude, roll, pitch, desired_acceleration};
}
