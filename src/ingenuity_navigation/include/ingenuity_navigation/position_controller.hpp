#ifndef INGENUITY_NAVIGATION__POSITION_CONTROLLER_HPP_
#define INGENUITY_NAVIGATION__POSITION_CONTROLLER_HPP_

#include "ingenuity_navigation/flight_math.hpp"

/**
 * @brief Converte o erro X/Y em uma ORIENTAÇÃO alvo do helicóptero.
 *
 * 1. PD de posição em coordenadas globais -> aceleração horizontal.
 * 2. Limitar aceleração para não pedir inclinações excessivas.
 * 3. Converter a direção do empuxo para roll/pitch do corpo.
 *
 * Não publica tópicos e não controla motores. É apenas matemática.
 */
class PositionController
{
public:
    struct Config
    {
        double kp = 0.45;       // (m/s²)/m
        double kd = 1.20;       // (m/s²)/(m/s)
        double max_tilt_rad = 0.08;
        double gravity = 9.81;
        // O SDF do Ingenuity usa <xyz>0 0 -1</xyz> nos dois rotores.
        double thrust_axis_z = -1.0;
    };

    struct Output
    {
        ingenuity::Quaternion target_orientation{};
        double roll_rad = 0.0;
        double pitch_rad = 0.0;
        ingenuity::Vector3 desired_acceleration_world{};
    };

    explicit PositionController(const Config &config);

    Output update(const ingenuity::Vector3 &target_world,
                  const ingenuity::Vector3 &current_world,
                  const ingenuity::Vector3 &velocity_world,
                  const ingenuity::Quaternion &hover_orientation,
                  double gain_scale = 1.0) const;

private:
    Config config_;
};

#endif
