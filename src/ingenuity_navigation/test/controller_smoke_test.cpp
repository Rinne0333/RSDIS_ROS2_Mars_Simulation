/**
 * @file controller_smoke_test.cpp
 * @brief Testes matemáticos e da máquina de estados; independentes de ROS/Gazebo.
 *
 * Nao validam sinais dos rotores, torque fisico de yaw nem ganhos em voo.
 */
#include "ingenuity_navigation/altitude_controller.hpp"
#include "ingenuity_navigation/attitude_controller.hpp"
#include "ingenuity_navigation/flight_math.hpp"
#include "ingenuity_navigation/mission_manager.hpp"
#include "ingenuity_navigation/motor_mixer.hpp"
#include "ingenuity_navigation/position_controller.hpp"
#include "ingenuity_navigation/yaw_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{
constexpr double kPi = 3.14159265358979323846;
bool near(double a, double b, double tol = 1e-4)
{
    return std::abs(a - b) < tol;
}
void firstTakeoff(MissionManager &m, const ingenuity::Quaternion &q)
{
    // O drone assenta em z=3.32 e fica sem velocidade ate capturar A.
    for (int i = 0; i <= 20; ++i)
    {
        m.update(i * 0.1, {0.0, -2.5, 3.32}, {}, {}, q);
    }
    assert(m.state() == MissionManager::State::TAKEOFF);
}
}

int main()
{
    AltitudeController alt(AltitudeController::Config{});
    assert(near(alt.update(5.0, 5.0, 0.0, 0.05), 19.62));

    // Somar e subtrair diferenciais conserva T (se não houver saturação).
    MotorMixer mixer(MotorMixer::Config{});
    const auto r = mixer.calculate(20.0, +1.5);
    assert(near(r.top_thrust, 11.5));
    assert(near(r.bottom_thrust, 8.5));
    assert(near(r.top_thrust + r.bottom_thrust, 20.0));

    // YawController: modulo de +/-180 graus, e amortecimento da velocidade.
    assert(near(YawController::wrapAngle(3.0*kPi), kPi) ||
           near(YawController::wrapAngle(3.0*kPi), -kPi));
    YawController yaw(YawController::Config{});
    assert(yaw.update(+0.1, 0.0, 0.0) > 0.0);
    assert(yaw.update(0.0, 0.0, +0.5) < 0.0);

    // O SDF tem rotor no eixo -Z e o corpo cerca de 180 graus invertido.
    const ingenuity::Quaternion q_hover{1.0, 0.0, 0.0, 0.0};
    const auto lift = ingenuity::rotate(q_hover, {0, 0, -1});
    assert(near(lift.z, +1.0));
    const auto q_turn = ingenuity::normalized(ingenuity::multiply(
        ingenuity::worldYawQuaternion(kPi / 2), q_hover));
    assert(near(ingenuity::headingWorld(q_turn), kPi / 2));
    // NOTA: w_body.z e negativo quando w_world.z e positivo (corpo invertido).
    assert(near(ingenuity::yawRateWorld(q_hover, {0,0,-1}), +1.0));

    PositionController pos(PositionController::Config{});
    const auto target = pos.update({3.0, 0.0, 5.0}, {0.0, 0.0, 5.0},
                                   {}, q_hover);
    const auto desire_lift = ingenuity::rotate(target.target_orientation,
                                               {0.0, 0.0, -1.0});
    assert(desire_lift.x > 0.0 && desire_lift.z > 0.0);

    // Teste eixo Y com a proa ja rodada para +Y.
    const auto target_y = pos.update({0.0, 3.0, 5.0}, {0.0, 0.0, 5.0},
                                     {}, q_turn);
    const auto lift_y = ingenuity::rotate(target_y.target_orientation,
                                          {0.0, 0.0, -1.0});
    assert(lift_y.y > 0.0);

    // Confirmar erro roll do attitude PD.
    AttitudeController attitude(AttitudeController::Config{});
    const auto q_roll = ingenuity::rollPitchQuaternion(0.05, 0.0);
    assert(attitude.update(q_roll, {}, {}).swashplate_2 > 0.0);

    MissionManager mission(MissionManager::Config{});
    firstTakeoff(mission, q_hover);
    assert(near(mission.destination().x, 3.0));

    // TAKEOFF -> HOVER_A apenas se altura E posicao E velocidade OK.
    mission.update(5.8, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::HOVER_A);
    mission.update(5.9, {0.0,-2.5,5.32}, {}, {}, q_hover);
    mission.update(8.0, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::ROTATE);
    assert(near(mission.targetPosition(8.0).x, 0.0)); // rodar sem deslocar
    mission.update(8.1, {0.0,-2.5,5.32}, {}, {}, q_hover);
    mission.update(9.3, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::NAVIGATE);

    // NAVIGATE deve fornecer trajetoria gradual, nao salto imediato a B.
    const auto nav_target = mission.targetPosition(9.4);
    assert(nav_target.x >= 0.0 && nav_target.x < 1.0);
    mission.update(12.0, {2.2,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::BRAKE);
    mission.update(12.1, {3.0,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::HOVER_B);
    mission.update(12.2, {3.0,-2.5,5.32}, {}, {}, q_hover);
    mission.update(14.4, {3.0,-2.5,5.32}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::LANDING);
    mission.update(21.0, {3.0,-2.5,3.33}, {}, {}, q_hover);
    assert(mission.state() == MissionManager::State::COMPLETE);

    // Missao com B a +Y: nao aceitar ROTATE se yaw continuar errado.
    MissionManager::Config config_y;
    config_y.goal_dx = 0.0;
    config_y.goal_dy = 3.0;
    MissionManager turning(config_y);
    firstTakeoff(turning, q_hover);
    turning.update(5.8, {0.0,-2.5,5.32}, {}, {}, q_hover);
    turning.update(5.9, {0.0,-2.5,5.32}, {}, {}, q_hover);
    turning.update(8.0, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(turning.state() == MissionManager::State::ROTATE);
    assert(near(turning.yawErrorWorld(q_hover), kPi/2));
    turning.update(8.1, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(turning.state() == MissionManager::State::ROTATE);
    turning.update(8.2, {0.0,-2.5,5.32}, {}, {}, q_turn);
    turning.update(9.5, {0.0,-2.5,5.32}, {}, {}, q_turn);
    assert(turning.state() == MissionManager::State::NAVIGATE);

    // Se o rotor nao conseguir produzir yaw, o tempo limite impede
    // avancar para NAVIGATE com heading errado.
    MissionManager no_yaw(config_y);
    firstTakeoff(no_yaw, q_hover);
    no_yaw.update(5.8, {0.0,-2.5,5.32}, {}, {}, q_hover);
    no_yaw.update(5.9, {0.0,-2.5,5.32}, {}, {}, q_hover);
    no_yaw.update(8.0, {0.0,-2.5,5.32}, {}, {}, q_hover);
    no_yaw.update(29.0, {0.0,-2.5,5.32}, {}, {}, q_hover);
    assert(no_yaw.state() == MissionManager::State::FAULT);

    std::cout << "controller_smoke_test: PASS\n";
    return 0;
}
