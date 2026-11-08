#ifndef INGENUITY_NAVIGATION__ALTITUDE_CONTROLLER_HPP_
#define INGENUITY_NAVIGATION__ALTITUDE_CONTROLLER_HPP_

/**
 * @brief PID de altitude. Não conhece ROS, Gazebo nem motores.
 *
 * Recebe a referência Z, a posição Z, a velocidade vertical no mundo e dt.
 * Devolve EMPUXO TOTAL em newtons (antes da distribuição pelos rotores).
 */
class AltitudeController
{
public:
    struct Config
    {
        double kp = 4.0;
        double ki = 0.0;
        double kd = 5.0;
        double hover_thrust = 19.62;  // 2 kg * 9.81 m/s² (modelo corrigido)
        double min_thrust = 0.0;
        double max_thrust = 70.0;
        double integral_limit = 5.0;
    };

    explicit AltitudeController(const Config &config);

    double update(double target_z, double current_z,
                  double world_velocity_z, double dt);
    void reset();

private:
    Config config_;
    double integral_error_ = 0.0;
};

#endif
