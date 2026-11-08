#ifndef INGENUITY_NAVIGATION__NAVIGATION_HPP_
#define INGENUITY_NAVIGATION__NAVIGATION_HPP_

/**
 * @file navigation.hpp
 * @brief Nó ROS 2: recebe odometria, coordena controladores e publica atuadores.
 *
 * Separação de responsabilidades:
 *   MissionManager       -> Em que etapa está a missão? Qual o destino agora?
 *   PositionController   -> Que inclinação permite chegar à posição X/Y?
 *   AltitudeController   -> Que empuxo total permite atingir a altitude Z?
 *   AttitudeController   -> Que referências enviar às duas swashplates?
 *   YawController        -> Diferenca de empuxo para tentar rodar em Z.
 *   MotorMixer           -> Quanto empuxo pedir a cada rotor?
 *   IngenuityNavigation  -> Parametrização, ROS 2, validação e segurança.
 */

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "ingenuity_navigation/altitude_controller.hpp"
#include "ingenuity_navigation/attitude_controller.hpp"
#include "ingenuity_navigation/flight_math.hpp"
#include "ingenuity_navigation/mission_manager.hpp"
#include "ingenuity_navigation/motor_mixer.hpp"
#include "ingenuity_navigation/position_controller.hpp"
#include "ingenuity_navigation/yaw_controller.hpp"

class IngenuityNavigation : public rclcpp::Node
{
public:
    IngenuityNavigation();
    ~IngenuityNavigation() override;

private:
    struct Settings
    {
        bool arm = false;              // false: publica sempre referências neutras
        double control_hz = 20.0;
        double odom_timeout_wall_s = 2.0;
        double max_odom_sample_gap_s = 0.75;
        double max_pose_jump_m = 1.5;
        int max_invalid_samples = 3;
        double swashplate_1_sign = 1.0;
        double swashplate_2_sign = 1.0;
        double max_swash_rad = 0.05;
        bool enable_yaw_control = true;
        double yaw_differential_sign = 1.0;
        double takeoff_xy_gain_scale = 0.35;
        double min_lift_axis_z = 0.65;
        std::string odom_frame = "nasa_ingenuity/odom";
        std::string body_frame = "nasa_ingenuity/base_footprint";
    };

    /** Uma amostra coerente. Velocidade linear já expressa no mundo. */
    struct OdometrySample
    {
        ingenuity::Vector3 position_world{};
        ingenuity::Vector3 velocity_world{};
        ingenuity::Vector3 angular_velocity_body{};
        ingenuity::Quaternion orientation{};
        double simulation_seconds = 0.0;
        std_msgs::msg::Header header{};
        std::uint64_t sequence = 0;
    };

    void loadParameters();
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);
    void controlLoop();
    void publishActuators(double top, double bottom, double sw1, double sw2);
    void publishNeutral();
    void publishMissionStatus(MissionManager::State state,
                              const ingenuity::Vector3 &target);
    void enterFault(const std::string &reason);

    // Configuração imutável durante a missão (alterar apenas no arranque).
    Settings settings_{};
    MissionManager::Config mission_config_{};
    AltitudeController::Config altitude_config_{};
    AttitudeController::Config attitude_config_{};
    PositionController::Config position_config_{};
    MotorMixer::Config mixer_config_{};
    YawController::Config yaw_config_{};

    // Controladores matemáticos: construídos após ler e validar parâmetros.
    std::unique_ptr<MissionManager> mission_;
    std::unique_ptr<AltitudeController> altitude_;
    std::unique_ptr<AttitudeController> attitude_;
    std::unique_ptr<PositionController> position_;
    std::unique_ptr<MotorMixer> mixer_;
    std::unique_ptr<YawController> yaw_;

    // Subsistema ROS 2.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr top_rotor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bottom_rotor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swashplate_1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swashplate_2_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    OdometrySample sample_{};
    bool odometry_received_ = false;
    int invalid_samples_ = 0;
    std::string last_invalid_reason_{};
    std::uint64_t processed_sequence_ = 0;
    double last_processed_simulation_s_ = -1.0;
    std::chrono::steady_clock::time_point last_valid_odom_wall_{};
    std::chrono::steady_clock::time_point last_log_wall_{};

    // Comandos mantidos entre duas mensagens de odometria válidas.
    MotorMixer::Output last_motors_{};
    AttitudeController::Output last_swashplates_{};
    bool fault_reported_ = false;
};

#endif
