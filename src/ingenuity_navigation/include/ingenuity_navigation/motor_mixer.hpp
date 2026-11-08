#ifndef INGENUITY_NAVIGATION__MOTOR_MIXER_HPP_
#define INGENUITY_NAVIGATION__MOTOR_MIXER_HPP_

/**
 * @brief Distribuição dos comandos entre os rotores coaxiais.
 *
 * Empuxo total mantém a altitude. A diferença tenta gerar yaw.
 *    TOP    = T * top_fraction       + yaw_differential_n
 *    BOTTOM = T * (1-top_fraction)   - yaw_differential_n
 *
 * A física do Gazebo ainda precisa de confirmar que a diferença produz yaw.
 */
class MotorMixer
{
public:
    struct Config
    {
        double top_fraction = 0.5;
        double max_thrust_per_rotor = 35.0;
    };

    struct Output
    {
        double top_thrust = 0.0;
        double bottom_thrust = 0.0;
    };

    explicit MotorMixer(const Config &config);
    Output calculate(double total_thrust, double yaw_differential_n = 0.0) const;

private:
    Config config_;
};
#endif
