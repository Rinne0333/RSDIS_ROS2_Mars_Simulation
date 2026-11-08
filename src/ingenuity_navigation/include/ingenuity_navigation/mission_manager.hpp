#ifndef INGENUITY_NAVIGATION__MISSION_MANAGER_HPP_
#define INGENUITY_NAVIGATION__MISSION_MANAGER_HPP_

#include "ingenuity_navigation/flight_math.hpp"
#include <string>

/**
 * @brief Sequenciador de uma missão A -> B, independente de ROS 2.
 *
 * Recebe amostras com tempo de simulacao; a maquina de estados apenas escolhe
 * destinos e transicoes. Nao controla diretamente nenhum motor.
 *
 * WAITING -> TAKEOFF -> HOVER_A -> ROTATE -> NAVIGATE -> BRAKE ->
 * HOVER_B -> LANDING -> COMPLETE
 *                          \-> FAULT (em qualquer estado ativo)
 *
 * ROTATE pode ser desligado via parâmetro, se o modelo nao possuir yaw.
 */
class MissionManager
{
public:
    enum class State
    {
        WAITING, TAKEOFF, HOVER_A, ROTATE, NAVIGATE, BRAKE,
        HOVER_B, LANDING, COMPLETE, FAULT
    };

    struct Config
    {
        double goal_dx = 3.0;             // B = A + (dx,dy) em odom [m]
        double goal_dy = 0.0;
        double flight_height = 2.0;       // altitude acima da cota de A [m]
        bool rotate_before_navigation = true;
        double settle_seconds = 1.5;
        double takeoff_speed = 0.55;      // rampa de referencia z [m/s]
        double landing_speed = 0.30;
        double hover_a_seconds = 2.0;     // estabilidade continua [s]
        double hover_b_seconds = 2.0;
        double rotate_hold_seconds = 1.0;
        double yaw_tolerance_deg = 5.0;
        double yaw_rate_tolerance = 0.12; // rad/s mundo
        double navigation_speed = 0.40;    // progressao gradual A->B [m/s]
        double max_target_lead_m = 0.65; // referencia no maximo 65 cm a frente
        double brake_radius = 1.0;        // comecar a travar a esta distancia
        double arrival_radius = 0.30;
        double arrival_speed = 0.25;
        double hover_a_radius = 0.45;
        double hover_a_speed = 0.30;
        double max_takeoff_drift = 2.0;
        double stage_timeout = 60.0;     // por estado, tempo sim [s]
        double rotate_timeout = 20.0;    // timeout especifico de ROTATE
        double max_xy_radius = 15.0;
        double max_overheight = 2.0;
    };

    explicit MissionManager(const Config &config);

    void update(double simulation_seconds,
                const ingenuity::Vector3 &position,
                const ingenuity::Vector3 &velocity_world,
                const ingenuity::Vector3 &angular_velocity_body,
                const ingenuity::Quaternion &orientation);
    void fail(const std::string &reason);

    State state() const { return state_; }
    bool active() const;
    bool finished() const;
    const std::string &faultReason() const { return fault_reason_; }
    const ingenuity::Vector3 &start() const { return start_; }
    const ingenuity::Vector3 &destination() const { return destination_; }
    const ingenuity::Quaternion &hoverOrientation() const { return hover_orientation_; }
    ingenuity::Quaternion headingOrientation() const;
    double destinationHeadingWorld() const { return destination_heading_world_; }
    double yawErrorWorld(const ingenuity::Quaternion &orientation) const;
    ingenuity::Vector3 targetPosition(double simulation_seconds) const;
    static const char *stateName(State state);

private:
    void transition(State next, double simulation_seconds);
    void updateSteadyTimer(bool steady, double t);
    bool steadyFor(double seconds, double t) const;

    Config config_;
    State state_ = State::WAITING;
    double stage_start_seconds_ = 0.0;
    double steady_since_seconds_ = -1.0;
    double landing_initial_z_ = 0.0;
    double initial_heading_world_ = 0.0;
    double destination_heading_world_ = 0.0;
    ingenuity::Vector3 start_{};
    ingenuity::Vector3 destination_{};
    ingenuity::Vector3 last_position_{};
    ingenuity::Quaternion hover_orientation_{};
    std::string fault_reason_{};
};
#endif
