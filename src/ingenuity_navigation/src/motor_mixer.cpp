#include "ingenuity_navigation/motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

MotorMixer::MotorMixer(const Config &config) : config_(config)
{
    if (!std::isfinite(config.top_fraction) ||
        !std::isfinite(config.max_thrust_per_rotor) ||
        config.top_fraction <= 0.0 || config.top_fraction >= 1.0 ||
        config.max_thrust_per_rotor <= 0.0 ||
        config.max_thrust_per_rotor > 100.0)
    {
        throw std::invalid_argument("Invalid rotor mixer configuration");
    }
}

MotorMixer::Output MotorMixer::calculate(double total_thrust,
                                         double yaw_differential_n) const
{
    if (!std::isfinite(total_thrust) || !std::isfinite(yaw_differential_n))
    {
        throw std::invalid_argument("Invalid motor-mixer inputs");
    }
    if (total_thrust <= 0.0)
    {
        return {};
    }

    const double top_nominal = total_thrust * config_.top_fraction;
    const double bottom_nominal = total_thrust * (1.0 - config_.top_fraction);

    // Nunca ultrapassar os limites físicos. Se o motor saturar, perde-se
    // capacidade de yaw e possivelmente empuxo. É visível na telemetria.
    return {
        std::clamp(top_nominal + yaw_differential_n, 0.0,
                   config_.max_thrust_per_rotor),
        std::clamp(bottom_nominal - yaw_differential_n, 0.0,
                   config_.max_thrust_per_rotor)
    };
}
