#include "ingenuity_navigation/navigation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;
using std::placeholders::_1;

// ============================================================================
// 1. INICIALIZAÇÃO
// ============================================================================
IngenuityNavigation::IngenuityNavigation()
    : rclcpp::Node("ingenuity_navigation")
{
    // Ler parâmetros primeiro: os objetos de controlo dependem destes valores.
    loadParameters();

    mission_ = std::make_unique<MissionManager>(mission_config_);
    altitude_ = std::make_unique<AltitudeController>(altitude_config_);
    attitude_ = std::make_unique<AttitudeController>(attitude_config_);
    position_ = std::make_unique<PositionController>(position_config_);
    mixer_ = std::make_unique<MotorMixer>(mixer_config_);
    yaw_ = std::make_unique<YawController>(yaw_config_);

    // Atuadores: std_msgs/Float64, tal como os remaps de mars.launch.xml.
    top_rotor_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/ingenuity/top_blades/thrust", 10);
    bottom_rotor_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/ingenuity/bottom_blades/thrust", 10);
    swashplate_1_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/ingenuity/swashplate_1/joint", 10);
    swashplate_2_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/ingenuity/swashplate_2/joint", 10);

    // Tópicos para seguir a missão com o terminal / RViz.
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "/ingenuity/navigation/state", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/ingenuity/navigation/target", 10);

    // SensorDataQoS aceita publishers de odometria BEST_EFFORT ou RELIABLE.
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ingenuity/odometry", rclcpp::SensorDataQoS(),
        std::bind(&IngenuityNavigation::odometryCallback, this, _1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / settings_.control_hz));

    // Timer de parede: continua a vigiar a receção mesmo com Gazebo pausado.
    // Os dt dos PID vêm dos timestamps da odometria, e NÃO deste timer.
    control_timer_ = create_wall_timer(
        period, std::bind(&IngenuityNavigation::controlLoop, this));

    RCLCPP_INFO(get_logger(),
        "Ingenuity A->B ready | arm=%s | offset=(%.2f, %.2f)m | height=%.2fm",
        settings_.arm ? "true" : "false", mission_config_.goal_dx,
        mission_config_.goal_dy, mission_config_.flight_height);
    RCLCPP_WARN(get_logger(),
        "Gazebo simulation only. Rotor/swashplate signs and flight gains require calibration.");
}

IngenuityNavigation::~IngenuityNavigation()
{
    // Último esforço para solicitar neutralidade num shutdown limpo.
    // SIGKILL / falha do processo não garantem que a mensagem seja entregue.
    if (top_rotor_pub_ && bottom_rotor_pub_ &&
        swashplate_1_pub_ && swashplate_2_pub_)
    {
        publishNeutral();
    }
}

// ============================================================================
// 2. PARÂMETROS ROS 2
// ============================================================================
void IngenuityNavigation::loadParameters()
{
    settings_.arm = declare_parameter<bool>("arm", false);
    settings_.control_hz = declare_parameter<double>("control_hz", 20.0);
    settings_.odom_timeout_wall_s =
        declare_parameter<double>("odom_timeout_wall_s", 2.0);
    settings_.max_odom_sample_gap_s =
        declare_parameter<double>("max_odom_sample_gap_s", 0.75);
    settings_.max_pose_jump_m =
        declare_parameter<double>("max_pose_jump_m", 1.5);
    settings_.max_invalid_samples =
        declare_parameter<int>("max_invalid_samples", 3);
    settings_.swashplate_1_sign =
        declare_parameter<double>("swashplate_1_sign", 1.0);
    settings_.swashplate_2_sign =
        declare_parameter<double>("swashplate_2_sign", 1.0);
    settings_.max_swash_rad =
        declare_parameter<double>("max_swash_rad", 0.05);
    settings_.enable_yaw_control =
        declare_parameter<bool>("enable_yaw_control", true);
    settings_.yaw_differential_sign =
        declare_parameter<double>("yaw_differential_sign", 1.0);
    settings_.takeoff_xy_gain_scale =
        declare_parameter<double>("takeoff_xy_gain_scale", 0.35);
    settings_.min_lift_axis_z =
        declare_parameter<double>("min_lift_axis_z", 0.65);
    settings_.odom_frame = declare_parameter<std::string>(
        "odom_frame", "nasa_ingenuity/odom");
    settings_.body_frame = declare_parameter<std::string>(
        "body_frame", "nasa_ingenuity/base_footprint");

    mission_config_.goal_dx = declare_parameter<double>("goal_dx", 3.0);
    mission_config_.goal_dy = declare_parameter<double>("goal_dy", 0.0);
    mission_config_.flight_height =
        declare_parameter<double>("flight_height", 2.0);
    mission_config_.settle_seconds =
        declare_parameter<double>("settle_seconds", 1.5);
    mission_config_.takeoff_speed =
        declare_parameter<double>("takeoff_speed", 0.55);
    mission_config_.landing_speed =
        declare_parameter<double>("landing_speed", 0.30);
    mission_config_.hover_a_seconds =
        declare_parameter<double>("hover_a_seconds", 1.5);
    mission_config_.hover_b_seconds =
        declare_parameter<double>("hover_b_seconds", 2.0);
    mission_config_.rotate_before_navigation =
        declare_parameter<bool>("rotate_before_navigation", true);
    mission_config_.rotate_hold_seconds =
        declare_parameter<double>("rotate_hold_seconds", 1.0);
    mission_config_.yaw_tolerance_deg =
        declare_parameter<double>("yaw_tolerance_deg", 5.0);
    mission_config_.yaw_rate_tolerance =
        declare_parameter<double>("yaw_rate_tolerance", 0.12);
    mission_config_.rotate_timeout =
        declare_parameter<double>("rotate_timeout", 20.0);
    mission_config_.navigation_speed =
        declare_parameter<double>("navigation_speed", 0.40);
    mission_config_.max_target_lead_m =
        declare_parameter<double>("max_target_lead_m", 0.65);
    mission_config_.brake_radius =
        declare_parameter<double>("brake_radius", 1.0);
    mission_config_.arrival_radius =
        declare_parameter<double>("arrival_radius", 0.30);
    mission_config_.arrival_speed =
        declare_parameter<double>("arrival_speed", 0.35);
    mission_config_.hover_a_radius =
        declare_parameter<double>("hover_a_radius", 0.45);
    mission_config_.hover_a_speed =
        declare_parameter<double>("hover_a_speed", 0.35);
    mission_config_.max_takeoff_drift =
        declare_parameter<double>("max_takeoff_drift", 2.0);
    mission_config_.stage_timeout =
        declare_parameter<double>("stage_timeout", 60.0);
    mission_config_.max_xy_radius =
        declare_parameter<double>("max_xy_radius", 15.0);
    mission_config_.max_overheight =
        declare_parameter<double>("max_overheight", 2.0);

    altitude_config_.kp = declare_parameter<double>("kp_z", 4.0);
    altitude_config_.ki = declare_parameter<double>("ki_z", 0.0);
    altitude_config_.kd = declare_parameter<double>("kd_z", 5.0);
    altitude_config_.hover_thrust =
        declare_parameter<double>("hover_total", 19.62);
    altitude_config_.min_thrust = 0.0;
    altitude_config_.max_thrust =
        declare_parameter<double>("max_total_thrust", 70.0);
    altitude_config_.integral_limit =
        declare_parameter<double>("integral_limit_z", 5.0);

    position_config_.kp = declare_parameter<double>("kp_xy", 0.45);
    position_config_.kd = declare_parameter<double>("kd_xy", 1.20);
    position_config_.max_tilt_rad =
        declare_parameter<double>("max_tilt_rad", 0.08);
    position_config_.gravity = declare_parameter<double>("gravity", 9.81);
    position_config_.thrust_axis_z =
        declare_parameter<double>("thrust_axis_z", -1.0);

    attitude_config_.kp = declare_parameter<double>("kp_att", 0.5);
    attitude_config_.kd = declare_parameter<double>("kd_att", 0.15);
    attitude_config_.max_swashplate_angle = settings_.max_swash_rad;

    mixer_config_.top_fraction =
        declare_parameter<double>("top_fraction", 0.5);
    mixer_config_.max_thrust_per_rotor =
        declare_parameter<double>("max_thrust_per_rotor", 35.0);

    // YawController: comando diferencial, em NEWTONS por rotor.
    yaw_config_.kp = declare_parameter<double>("kp_yaw", 1.4);
    yaw_config_.kd = declare_parameter<double>("kd_yaw", 0.5);
    yaw_config_.max_diff_n =
        declare_parameter<double>("max_yaw_diff_n", 1.5);

    // Os restantes parâmetros são validados pelos respetivos controladores.
    const auto sign_ok = [](double v) { return v == -1.0 || v == 1.0; };
    if (mission_config_.rotate_before_navigation &&
        !settings_.enable_yaw_control)
    {
        throw std::invalid_argument(
            "rotate_before_navigation requires enable_yaw_control=true");
    }
    if (!sign_ok(settings_.yaw_differential_sign) ||
        !std::isfinite(settings_.takeoff_xy_gain_scale) ||
        settings_.takeoff_xy_gain_scale < 0.0 ||
        settings_.takeoff_xy_gain_scale > 1.0 ||
        !sign_ok(settings_.swashplate_1_sign) ||
        !sign_ok(settings_.swashplate_2_sign) ||
        !std::isfinite(settings_.control_hz) ||
        settings_.control_hz < 1.0 || settings_.control_hz > 200.0 ||
        !std::isfinite(settings_.odom_timeout_wall_s) ||
        settings_.odom_timeout_wall_s < 0.2 ||
        !std::isfinite(settings_.max_odom_sample_gap_s) ||
        settings_.max_odom_sample_gap_s < 0.05 ||
        !std::isfinite(settings_.max_pose_jump_m) ||
        settings_.max_pose_jump_m <= 0.0 ||
        settings_.max_invalid_samples < 1 ||
        !std::isfinite(settings_.min_lift_axis_z) ||
        settings_.min_lift_axis_z < 0.0 ||
        settings_.min_lift_axis_z > 1.0 ||
        settings_.odom_frame.empty() || settings_.body_frame.empty())
    {
        throw std::invalid_argument("Invalid navigation settings");
    }
}

// ============================================================================
// 3. AQUISIÇÃO E VALIDAÇÃO DA ODOMETRIA
// ============================================================================
void IngenuityNavigation::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
{
    const auto &pose = message->pose.pose;
    const auto &twist = message->twist.twist;
    const ingenuity::Vector3 p{pose.position.x, pose.position.y, pose.position.z};
    const ingenuity::Vector3 v_body{twist.linear.x, twist.linear.y,
                                    twist.linear.z};
    const ingenuity::Vector3 w_body{twist.angular.x, twist.angular.y,
                                    twist.angular.z};
    const ingenuity::Quaternion q_raw{
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w};

    // A pose é expressa no frame odom e o twist no child_frame_id.
    // Se os frames não forem os esperados, não podemos misturá-los.
    const bool frames_ok =
        message->header.frame_id == settings_.odom_frame &&
        message->child_frame_id == settings_.body_frame;

    if (!frames_ok || !ingenuity::finite(p) ||
        !ingenuity::finite(v_body) || !ingenuity::finite(w_body) ||
        !ingenuity::finite(q_raw))
    {
        last_invalid_reason_ = !frames_ok ?
            "unexpected frame_id or child_frame_id" :
            "non-finite position, velocity or orientation";
        ++invalid_samples_;
        if (invalid_samples_ == 1 && !frames_ok)
        {
            RCLCPP_WARN(get_logger(),
                "Unexpected odometry frames: '%s' / '%s' (expected '%s' / '%s')",
                message->header.frame_id.c_str(),
                message->child_frame_id.c_str(),
                settings_.odom_frame.c_str(), settings_.body_frame.c_str());
        }
        return;
    }

    ingenuity::Quaternion q;
    try
    {
        q = ingenuity::normalized(q_raw);
    }
    catch (const std::invalid_argument &)
    {
        last_invalid_reason_ = "invalid quaternion";
        ++invalid_samples_;
        return;
    }

    const double stamp_s =
        static_cast<double>(message->header.stamp.sec) +
        1e-9 * static_cast<double>(message->header.stamp.nanosec);

    if (!std::isfinite(stamp_s) || stamp_s < 0.0)
    {
        last_invalid_reason_ = "invalid timestamp";
        ++invalid_samples_;
        return;
    }

    // Verificamos regressões de tempo, como num reset do Gazebo.
    if (odometry_received_ && stamp_s < sample_.simulation_seconds - 1e-6)
    {
        enterFault("Odometry timestamp moved backwards (Gazebo reset?)");
        return;
    }

    // Uma mensagem repetida não deve integrar novamente os PID.
    if (odometry_received_ && stamp_s <= sample_.simulation_seconds + 1e-6)
    {
        return;
    }

    const ingenuity::Vector3 v_world = ingenuity::rotate(q, v_body);

    // Rejeitar saltos obviamente incompatíveis com um pequeno helicóptero.
    if (odometry_received_ &&
        ingenuity::norm(p - sample_.position_world) > settings_.max_pose_jump_m)
    {
        last_invalid_reason_ = "odometry position jumped too far";
        ++invalid_samples_;
        return;
    }

    sample_.position_world = p;
    sample_.velocity_world = v_world;
    sample_.angular_velocity_body = w_body;
    sample_.orientation = q;
    sample_.simulation_seconds = stamp_s;
    sample_.header = message->header;
    ++sample_.sequence;

    odometry_received_ = true;
    invalid_samples_ = 0;
    last_invalid_reason_.clear();
    last_valid_odom_wall_ = std::chrono::steady_clock::now();
}

// ============================================================================
// 4. PUBLICAÇÃO DOS ATUADORES E TELEMETRIA
// ============================================================================
void IngenuityNavigation::publishActuators(
    double top, double bottom, double sw1, double sw2)
{
    // Última barreira: nunca publicar NaN ou infinito para Gazebo.
    if (!std::isfinite(top) || !std::isfinite(bottom) ||
        !std::isfinite(sw1) || !std::isfinite(sw2))
    {
        top = bottom = sw1 = sw2 = 0.0;
    }

    std_msgs::msg::Float64 message;
    message.data = std::clamp(top, 0.0, mixer_config_.max_thrust_per_rotor);
    top_rotor_pub_->publish(message);
    message.data = std::clamp(bottom, 0.0, mixer_config_.max_thrust_per_rotor);
    bottom_rotor_pub_->publish(message);
    message.data = std::clamp(sw1, -settings_.max_swash_rad,
                              settings_.max_swash_rad);
    swashplate_1_pub_->publish(message);
    message.data = std::clamp(sw2, -settings_.max_swash_rad,
                              settings_.max_swash_rad);
    swashplate_2_pub_->publish(message);
}

void IngenuityNavigation::publishNeutral()
{
    last_motors_ = {};
    last_swashplates_ = {};
    publishActuators(0.0, 0.0, 0.0, 0.0);
}

void IngenuityNavigation::publishMissionStatus(
    MissionManager::State state, const ingenuity::Vector3 &target)
{
    std_msgs::msg::String state_message;
    state_message.data = MissionManager::stateName(state);
    state_pub_->publish(state_message);

    geometry_msgs::msg::PointStamped point_message;
    point_message.header = sample_.header;
    point_message.point.x = target.x;
    point_message.point.y = target.y;
    point_message.point.z = target.z;
    target_pub_->publish(point_message);
}

void IngenuityNavigation::enterFault(const std::string &reason)
{
    if (mission_ && !mission_->finished())
    {
        mission_->fail(reason);
    }
    if (!fault_reported_)
    {
        RCLCPP_ERROR(get_logger(), "MISSION FAULT: %s", reason.c_str());
        fault_reported_ = true;
    }
    if (altitude_)
    {
        altitude_->reset();
    }
    if (state_pub_ && mission_)
    {
        std_msgs::msg::String msg;
        msg.data = MissionManager::stateName(mission_->state());
        state_pub_->publish(msg);
    }
    if (top_rotor_pub_ && bottom_rotor_pub_ &&
        swashplate_1_pub_ && swashplate_2_pub_)
    {
        publishNeutral();
    }
}

// ============================================================================
// 5. CICLO PRINCIPAL (20 Hz por omissão)
// ============================================================================
void IngenuityNavigation::controlLoop()
{
    const auto now_wall = std::chrono::steady_clock::now();

    // Desarmado é o estado predefinido, mesmo que a odometria esteja presente.
    if (!settings_.arm)
    {
        publishNeutral();
        return;
    }

    if (mission_->finished())
    {
        publishNeutral();
        return;
    }

    // Ainda não existe uma amostra? Não ligar rotores.
    if (!odometry_received_)
    {
        publishNeutral();
        return;
    }

    // Pior caso: houve interrupção de /ingenuity/odometry ou pausa do Gazebo.
    const double age_wall = std::chrono::duration<double>(
        now_wall - last_valid_odom_wall_).count();
    if (age_wall > settings_.odom_timeout_wall_s)
    {
        enterFault("Odometry stopped arriving (wall-clock timeout)");
        return;
    }

    if (invalid_samples_ >= settings_.max_invalid_samples)
    {
        enterFault("Too many invalid odometry messages: " + last_invalid_reason_);
        return;
    }

    // Evitar executar o PID mais do que uma vez sobre o mesmo sensor sample.
    if (sample_.sequence == processed_sequence_)
    {
        if (mission_->active())
        {
            publishActuators(last_motors_.top_thrust,
                             last_motors_.bottom_thrust,
                             last_swashplates_.swashplate_1,
                             last_swashplates_.swashplate_2);
        }
        else
        {
            publishNeutral();
        }
        return;
    }

    const double t = sample_.simulation_seconds;
    double dt = 1.0 / settings_.control_hz;
    if (last_processed_simulation_s_ >= 0.0)
    {
        dt = t - last_processed_simulation_s_;
        if (dt <= 0.0 || dt > settings_.max_odom_sample_gap_s)
        {
            if (mission_->active())
            {
                enterFault("Odometry timestamp gap during flight");
                return;
            }
            // Aguardar por uma amostra seguinte antes de iniciar a missão.
            altitude_->reset();
            last_processed_simulation_s_ = t;
            processed_sequence_ = sample_.sequence;
            publishNeutral();
            return;
        }
    }
    last_processed_simulation_s_ = t;
    processed_sequence_ = sample_.sequence;

    const MissionManager::State previous_state = mission_->state();
    try
    {
        mission_->update(t, sample_.position_world, sample_.velocity_world,
                         sample_.angular_velocity_body, sample_.orientation);
    }
    catch (const std::exception &error)
    {
        enterFault(std::string("Mission update: ") + error.what());
        return;
    }

    const auto state = mission_->state();
    const auto target = mission_->targetPosition(t);

    // Publica telemetria em cada amostra nova (status e target).
    publishMissionStatus(state, target);

    if (state != previous_state)
    {
        RCLCPP_INFO(get_logger(), "MISSION STATE: %s",
                    MissionManager::stateName(state));
        if (state == MissionManager::State::TAKEOFF)
        {
            RCLCPP_INFO(get_logger(),
                "A=(%.2f, %.2f, %.2f) | B=(%.2f, %.2f, %.2f)",
                mission_->start().x, mission_->start().y, mission_->start().z,
                mission_->destination().x, mission_->destination().y,
                mission_->destination().z);
        }
    }

    if (state == MissionManager::State::FAULT)
    {
        enterFault(mission_->faultReason());
        return;
    }
    if (state == MissionManager::State::COMPLETE)
    {
        publishNeutral();
        RCLCPP_INFO(get_logger(), "A->B mission COMPLETE. Rotors OFF.");
        return;
    }
    if (!mission_->active())
    {
        altitude_->reset();
        publishNeutral();
        return;
    }

    // O SDF tem eixos das hélices em -Z; a orientação inicial tem ~180° em X.
    // A referência de hover deve transformar o empuxo local para cima no mundo.
    const auto lift_world = ingenuity::rotate(
        mission_->hoverOrientation(), {0.0, 0.0, position_config_.thrust_axis_z});
    if (lift_world.z < settings_.min_lift_axis_z)
    {
        enterFault("Initial hover attitude does not point thrust upwards");
        return;
    }

    // Verificar também se o helicóptero não se inclinou excessivamente.
    const auto current_lift = ingenuity::rotate(
        sample_.orientation, {0.0, 0.0, position_config_.thrust_axis_z});
    if (current_lift.z < settings_.min_lift_axis_z)
    {
        enterFault("Excessive attitude tilt: thrust no longer points upwards");
        return;
    }

    try
    {
        // (1) Z -> força total. A velocidade vertical está em odom, não body.
        const double total_thrust = altitude_->update(
            target.z, sample_.position_world.z, sample_.velocity_world.z, dt);

        // (2) Direcao desejada para a proa do helicóptero.
        // Durante a subida e HOVER_A mantemos o heading inicial.
        // Apenas em ROTATE e nos estados posteriores apontamos para B.
        const bool heading_to_b =
            mission_config_.rotate_before_navigation && (
            state == MissionManager::State::ROTATE ||
            state == MissionManager::State::NAVIGATE ||
            state == MissionManager::State::BRAKE ||
            state == MissionManager::State::HOVER_B ||
            state == MissionManager::State::LANDING);
        const auto heading_orientation = heading_to_b ?
            mission_->headingOrientation() : mission_->hoverOrientation();

        // (3) XY: manter A durante TAKEOFF/HOVER_A/ROTATE.
        // Em NAVIGATE, a referencia avança gradualmente para B.
        // Um ganho XY menor na subida reduz correcoes bruscas no solo.
        const double xy_gain = state == MissionManager::State::TAKEOFF ?
            settings_.takeoff_xy_gain_scale : 1.0;
        const auto xy = position_->update(
            target, sample_.position_world, sample_.velocity_world,
            heading_orientation, xy_gain);

        // (4) Estabilizacao de roll/pitch, dois swashplates.
        const auto attitude_commands = attitude_->update(
            xy.target_orientation, sample_.orientation,
            sample_.angular_velocity_body);

        // (5) Yaw: o corpo esta invertido ~180 graus em X, por isso
        // medimos heading e yaw-rate no MUNDO (nao angular.z do corpo).
        const double current_heading =
            ingenuity::headingWorld(sample_.orientation);
        const double yaw_rate_world = ingenuity::yawRateWorld(
            sample_.orientation, sample_.angular_velocity_body);
        const double target_heading = heading_to_b ?
            mission_->destinationHeadingWorld() :
            ingenuity::headingWorld(mission_->hoverOrientation());
        const double yaw_error = YawController::wrapAngle(
            target_heading - current_heading);
        const double yaw_diff = settings_.enable_yaw_control ?
            settings_.yaw_differential_sign * yaw_->update(
                target_heading, current_heading, yaw_rate_world) : 0.0;

        // (6) MotorMixer distribui T e diferencial de yaw.
        // Se o Gazebo nao produzir binario com diferenca de empuxo,
        // ROTATE termina por timeout. Nao ha rotacao ficticia.
        last_motors_ = mixer_->calculate(total_thrust, yaw_diff);
        last_swashplates_.swashplate_1 = std::clamp(
            attitude_commands.swashplate_1 * settings_.swashplate_1_sign,
            -settings_.max_swash_rad, settings_.max_swash_rad);
        last_swashplates_.swashplate_2 = std::clamp(
            attitude_commands.swashplate_2 * settings_.swashplate_2_sign,
            -settings_.max_swash_rad, settings_.max_swash_rad);

        // (5) Aplicar os quatro comandos ao Gazebo.
        publishActuators(last_motors_.top_thrust,
                         last_motors_.bottom_thrust,
                         last_swashplates_.swashplate_1,
                         last_swashplates_.swashplate_2);

        // Logs no máximo a cada ~1 s de relógio real.
        if (now_wall - last_log_wall_ > 1s)
        {
            last_log_wall_ = now_wall;
            RCLCPP_INFO(get_logger(),
                "%s | XYZ=(%.2f, %.2f, %.2f) | setpoint=(%.2f, %.2f, %.2f) "
                "| Vz=%.2f | Vxy=%.2f | ErrXY=%.2f "
                "| T=%.2f N | SW=(%.3f, %.3f) | yaw_err=%.1fdeg "
                "| diffYaw=%.2fN | rotors=(%.2f,%.2f)",
                MissionManager::stateName(state),
                sample_.position_world.x, sample_.position_world.y,
                sample_.position_world.z,
                target.x, target.y, target.z,
                sample_.velocity_world.z,
                ingenuity::horizontalNorm(sample_.velocity_world),
                ingenuity::horizontalNorm(target - sample_.position_world),
                total_thrust,
                last_swashplates_.swashplate_1,
                last_swashplates_.swashplate_2,
                yaw_error * 180.0 / 3.14159265358979323846,
                yaw_diff, last_motors_.top_thrust,
                last_motors_.bottom_thrust);
        }
    }
    catch (const std::exception &error)
    {
        enterFault(std::string("Controller exception: ") + error.what());
    }
}
