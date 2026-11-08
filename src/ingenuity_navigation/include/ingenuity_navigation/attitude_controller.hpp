#ifndef INGENUITY_NAVIGATION__ATTITUDE_CONTROLLER_HPP_
#define INGENUITY_NAVIGATION__ATTITUDE_CONTROLLER_HPP_

#include "ingenuity_navigation/flight_math.hpp"

/**
 * @brief Controlo PD de roll/pitch em coordenadas locais do corpo.
 *
 * Recebe quaternion de referência e quaternion atual (corpo -> mundo).
 * O erro é calculado no corpo atual e a velocidade angular já está
 * expressa nesse mesmo referencial (convenção nav_msgs/Odometry).
 *
 * Saída: referências angulares [rad] para os DOIS swashplates.
 * Swashplate 1 = eixo mecânico Y; Swashplate 2 = eixo mecânico X.
 * Os sinais físicos são parâmetros configuráveis no nó principal.
 */
class AttitudeController
{
public:
    using Quaternion = ingenuity::Quaternion;
    using Vector3 = ingenuity::Vector3;

    struct Config
    {
        double kp = 0.5;
        double kd = 0.15;
        double max_swashplate_angle = 0.05;
    };

    struct Output
    {
        double swashplate_1 = 0.0;
        double swashplate_2 = 0.0;
    };

    explicit AttitudeController(const Config &config);
    Output update(const Quaternion &target, const Quaternion &current,
                  const Vector3 &body_angular_velocity) const;

private:
    Config config_;
};

#endif
