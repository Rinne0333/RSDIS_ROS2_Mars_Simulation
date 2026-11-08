#include "ingenuity_navigation/mission_manager.hpp"
#include "ingenuity_navigation/yaw_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

MissionManager::MissionManager(const Config &c) : config_(c)
{
    const double values[] = {
        c.goal_dx, c.goal_dy, c.flight_height, c.settle_seconds,
        c.takeoff_speed, c.landing_speed, c.hover_a_seconds, c.hover_b_seconds,
        c.rotate_hold_seconds, c.yaw_tolerance_deg, c.yaw_rate_tolerance,
        c.navigation_speed, c.max_target_lead_m, c.brake_radius, c.arrival_radius, c.arrival_speed, c.hover_a_radius,
        c.hover_a_speed, c.max_takeoff_drift, c.stage_timeout,
        c.rotate_timeout, c.max_xy_radius, c.max_overheight
    };
    for (const double value : values)
    {
        if (!std::isfinite(value))
            throw std::invalid_argument("Non-finite mission parameter");
    }
    if (c.flight_height <= 0.0 || c.flight_height > 10.0 ||
        c.settle_seconds < 0.0 || c.takeoff_speed <= 0.0 ||
        c.landing_speed <= 0.0 || c.hover_a_seconds < 0.0 ||
        c.hover_b_seconds < 0.0 || c.rotate_hold_seconds < 0.0 ||
        c.yaw_tolerance_deg <= 0.0 || c.yaw_tolerance_deg >= 90.0 ||
        c.yaw_rate_tolerance <= 0.0 || c.navigation_speed <= 0.0 || c.max_target_lead_m <= 0.0 ||
        c.brake_radius <= c.arrival_radius ||
        c.arrival_radius <= 0.0 || c.arrival_speed <= 0.0 ||
        c.hover_a_radius <= 0.0 || c.hover_a_speed <= 0.0 ||
        c.max_takeoff_drift <= c.hover_a_radius ||
        c.max_takeoff_drift >= c.max_xy_radius ||
        c.stage_timeout <= 0.0 || c.rotate_timeout <= 0.0 ||
        c.max_xy_radius <= 0.0 || c.max_overheight <= 0.0 ||
        std::hypot(c.goal_dx, c.goal_dy) >
            c.max_xy_radius - c.arrival_radius)
    {
        throw std::invalid_argument("Invalid mission parameter range");
    }
}

const char *MissionManager::stateName(State s)
{
    switch (s)
    {
    case State::WAITING: return "WAITING";
    case State::TAKEOFF: return "TAKEOFF";
    case State::HOVER_A: return "HOVER_A";
    case State::ROTATE: return "ROTATE";
    case State::NAVIGATE: return "NAVIGATE";
    case State::BRAKE: return "BRAKE";
    case State::HOVER_B: return "HOVER_B";
    case State::LANDING: return "LANDING";
    case State::COMPLETE: return "COMPLETE";
    case State::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}

bool MissionManager::active() const
{
    return state_ != State::WAITING && state_ != State::COMPLETE &&
           state_ != State::FAULT;
}

bool MissionManager::finished() const
{
    return state_ == State::COMPLETE || state_ == State::FAULT;
}

void MissionManager::fail(const std::string &reason)
{
    if (!finished())
    {
        state_ = State::FAULT;
        fault_reason_ = reason;
    }
}

void MissionManager::transition(State next, double t)
{
    state_ = next;
    stage_start_seconds_ = t;
    steady_since_seconds_ = -1.0;
}

void MissionManager::updateSteadyTimer(bool steady, double t)
{
    if (!steady) steady_since_seconds_ = -1.0;
    else if (steady_since_seconds_ < 0.0) steady_since_seconds_ = t;
}

bool MissionManager::steadyFor(double duration, double t) const
{
    return steady_since_seconds_ >= 0.0 &&
           t - steady_since_seconds_ >= duration;
}

/** Quaternion do corpo desejado quando aponta na direcao de B.
 * A rotacao e feita no Z do mundo, ANTES de q_hover. O corpo tem ~180 graus
 * em X; rodar em torno do seu Z local daria sinal de yaw inverso.
 */
ingenuity::Quaternion MissionManager::headingOrientation() const
{
    const double delta = YawController::wrapAngle(
        destination_heading_world_ - initial_heading_world_);
    return ingenuity::normalized(ingenuity::multiply(
        ingenuity::worldYawQuaternion(delta), hover_orientation_));
}

double MissionManager::yawErrorWorld(
    const ingenuity::Quaternion &orientation) const
{
    return YawController::wrapAngle(
        destination_heading_world_ - ingenuity::headingWorld(orientation));
}

void MissionManager::update(
    double t, const ingenuity::Vector3 &p, const ingenuity::Vector3 &v,
    const ingenuity::Vector3 &w_body, const ingenuity::Quaternion &q)
{
    if (finished()) return;
    if (!std::isfinite(t) || !ingenuity::finite(p) ||
        !ingenuity::finite(v) || !ingenuity::finite(w_body) ||
        !ingenuity::finite(q))
    {
        fail("Invalid mission sample");
        return;
    }

    last_position_ = p;
    if (state_ == State::WAITING)
    {
        // Nao capturar a posicao de spawn enquanto o drone ainda cai.
        const bool stable = ingenuity::norm(v) < 0.08 &&
                            ingenuity::norm(w_body) < 0.15;
        updateSteadyTimer(stable, t);
        if (steadyFor(config_.settle_seconds, t))
        {
            start_ = p;
            destination_ = {p.x + config_.goal_dx, p.y + config_.goal_dy, p.z};
            hover_orientation_ = ingenuity::normalized(q);
            initial_heading_world_ = ingenuity::headingWorld(q);
            destination_heading_world_ =
                std::hypot(config_.goal_dx, config_.goal_dy) > 1e-6 ?
                    std::atan2(config_.goal_dy, config_.goal_dx) :
                    initial_heading_world_;
            transition(State::TAKEOFF, t);
        }
        return;
    }

    const double elapsed = t - stage_start_seconds_;
    if (elapsed < -1e-6)
    {
        fail("Simulation time moved backwards");
        return;
    }
    const double timeout = state_ == State::ROTATE ?
        config_.rotate_timeout : config_.stage_timeout;
    if (elapsed > timeout)
    {
        fail(std::string("Timeout in ") + stateName(state_) +
             (state_ == State::ROTATE ? ": yaw actuation not confirmed" : ""));
        return;
    }

    const double distance_a = ingenuity::horizontalNorm(p - start_);
    const double distance_b = ingenuity::horizontalNorm(p - destination_);
    const double v_xy = ingenuity::horizontalNorm(v);
    if (distance_a > config_.max_xy_radius ||
        p.z < start_.z - 0.8 ||
        p.z > start_.z + config_.flight_height + config_.max_overheight)
    {
        fail("Outside mission geofence");
        return;
    }

    const bool stable_a = distance_a <= config_.hover_a_radius &&
                          v_xy <= config_.hover_a_speed;
    const bool stable_b = distance_b <= config_.arrival_radius &&
                          v_xy <= config_.arrival_speed;
    const double target_z = start_.z + config_.flight_height;
    const bool at_height = std::abs(p.z - target_z) <= 0.20 &&
                           std::abs(v.z) < 0.35;

    switch (state_)
    {
    case State::TAKEOFF:
        if (distance_a > config_.max_takeoff_drift)
        {
            fail("Takeoff XY drift exceeded limit: verify swashplates");
            break;
        }
        if (at_height && stable_a)
        {
            transition(State::HOVER_A, t);
        }
        break;

    case State::HOVER_A:
        if (distance_a > config_.max_takeoff_drift)
        {
            fail("Excessive XY drift at A");
            break;
        }
        updateSteadyTimer(stable_a && at_height, t);
        if (steadyFor(config_.hover_a_seconds, t))
        {
            if (std::hypot(config_.goal_dx, config_.goal_dy) <
                config_.arrival_radius)
            {
                transition(State::HOVER_B, t);
            }
            else
            {
                transition(config_.rotate_before_navigation ?
                    State::ROTATE : State::NAVIGATE, t);
            }
        }
        break;

    case State::ROTATE:
    {
        // Coordenadas XY e Z mantidas durante a rotacao.
        if (distance_a > config_.max_takeoff_drift)
        {
            fail("Excessive XY drift during rotation");
            break;
        }
        const double yaw_error = std::abs(yawErrorWorld(q));
        const double yaw_rate_world = std::abs(
            ingenuity::yawRateWorld(q, w_body));
        const bool aligned = yaw_error <= config_.yaw_tolerance_deg * kPi / 180.0 &&
                             yaw_rate_world <= config_.yaw_rate_tolerance;
        updateSteadyTimer(aligned && stable_a && at_height, t);
        if (steadyFor(config_.rotate_hold_seconds, t))
        {
            transition(State::NAVIGATE, t);
        }
        break;
    }

    case State::NAVIGATE:
        if (distance_b <= config_.brake_radius)
        {
            transition(State::BRAKE, t);
        }
        break;

    case State::BRAKE:
        // Um controlador PD horizontal trava pela velocidade: v_target=0.
        // BRAKE torna explicita a fase final da aproximacao.
        if (stable_b && at_height)
        {
            transition(State::HOVER_B, t);
        }
        break;

    case State::HOVER_B:
        updateSteadyTimer(stable_b && at_height, t);
        if (steadyFor(config_.hover_b_seconds, t))
        {
            landing_initial_z_ = p.z;
            transition(State::LANDING, t);
        }
        break;

    case State::LANDING:
        if (distance_b > config_.max_takeoff_drift)
        {
            fail("Landing drift away from B");
            break;
        }
        // Modelo assume terreno plano com a mesma cota de A e B.
        if (elapsed >= 1.0 && p.z <= start_.z + 0.14 &&
            std::abs(v.z) < 0.25 && v_xy < config_.arrival_speed)
        {
            transition(State::COMPLETE, t);
        }
        break;

    case State::WAITING:
    case State::COMPLETE:
    case State::FAULT:
        break;
    }
}

ingenuity::Vector3 MissionManager::targetPosition(double t) const
{
    const double flight_z = start_.z + config_.flight_height;
    switch (state_)
    {
    case State::TAKEOFF:
        return {start_.x, start_.y,
                std::min(flight_z, start_.z + config_.takeoff_speed *
                         std::max(0.0, t - stage_start_seconds_))};

    case State::HOVER_A:
    case State::ROTATE:
        return {start_.x, start_.y, flight_z};

    case State::NAVIGATE:
    {
        // Referencia com progressao limitada: em vez de saltar 3 m
        // instantaneamente, avancar ao longo de A->B a velocidade constante.
        const double length = std::hypot(config_.goal_dx, config_.goal_dy);
        if (length < 1e-9) return {start_.x, start_.y, flight_z};
        const double ux = config_.goal_dx / length;
        const double uy = config_.goal_dy / length;
        const double measured_progress =
            (last_position_.x - start_.x) * ux +
            (last_position_.y - start_.y) * uy;
        const double time_progress = config_.navigation_speed *
            std::max(0.0, t - stage_start_seconds_);
        const double progress = std::clamp(
            std::min(time_progress, measured_progress + config_.max_target_lead_m),
            0.0, length);
        return {start_.x + ux * progress, start_.y + uy * progress, flight_z};
    }

    case State::BRAKE:
    case State::HOVER_B:
        return {destination_.x, destination_.y, flight_z};

    case State::LANDING:
        return {destination_.x, destination_.y,
                std::max(start_.z + 0.03, landing_initial_z_ -
                    config_.landing_speed * std::max(0.0, t - stage_start_seconds_))};

    case State::WAITING:
    case State::COMPLETE:
    case State::FAULT:
        return start_;
    }
    return start_;
}
