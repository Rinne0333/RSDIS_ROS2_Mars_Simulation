#ifndef INGENUITY_NAVIGATION__FLIGHT_MATH_HPP_
#define INGENUITY_NAVIGATION__FLIGHT_MATH_HPP_

/**
 * @file flight_math.hpp
 * @brief Pequena biblioteca matemática independente do ROS 2.
 *
 * Convenções usadas em todo o package:
 *   - Posição e velocidade de navegação: referencial de odometria (m, m/s).
 *   - Quaternion q: orientação do corpo relativamente à odometria.
 *   - Velocidade angular: referencial do corpo (rad/s).
 *   - Quaternion: componentes (x, y, z, w), como em geometry_msgs.
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ingenuity
{
struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

inline Vector3 operator+(const Vector3 &a, const Vector3 &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vector3 operator-(const Vector3 &a, const Vector3 &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vector3 operator*(const Vector3 &v, double factor)
{
    return {v.x * factor, v.y * factor, v.z * factor};
}

inline bool finite(const Vector3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool finite(const Quaternion &q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w);
}

inline double horizontalNorm(const Vector3 &v)
{
    return std::hypot(v.x, v.y);
}

inline double norm(const Vector3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline Quaternion normalized(const Quaternion &q)
{
    if (!finite(q))
    {
        throw std::invalid_argument("Non-finite quaternion");
    }

    const double magnitude = std::sqrt(
        q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    if (magnitude < 1e-9)
    {
        throw std::invalid_argument("Zero-length quaternion");
    }

    return {q.x / magnitude, q.y / magnitude,
            q.z / magnitude, q.w / magnitude};
}

inline Quaternion conjugate(const Quaternion &q)
{
    return {-q.x, -q.y, -q.z, q.w};
}

inline Quaternion multiply(const Quaternion &a, const Quaternion &b)
{
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

/** Roda um vetor expresso no corpo para o referencial da odometria. */
inline Vector3 rotate(const Quaternion &q, const Vector3 &v)
{
    const Quaternion unit = normalized(q);
    const Quaternion vector_quaternion{v.x, v.y, v.z, 0.0};
    const Quaternion result = multiply(
        multiply(unit, vector_quaternion), conjugate(unit));
    return {result.x, result.y, result.z};
}

/** Quaternion para uma rotação local: primeiro pitch, depois roll. */
inline Quaternion rollPitchQuaternion(double roll, double pitch)
{
    const Quaternion q_roll{
        std::sin(roll / 2.0), 0.0, 0.0, std::cos(roll / 2.0)};
    const Quaternion q_pitch{
        0.0, std::sin(pitch / 2.0), 0.0, std::cos(pitch / 2.0)};
    return multiply(q_roll, q_pitch);
}

/** Rotacao em torno do Z do MUNDO; nao do Z do corpo invertido. */
inline Quaternion worldYawQuaternion(double yaw_rad)
{
    return {0.0, 0.0, std::sin(yaw_rad / 2.0),
            std::cos(yaw_rad / 2.0)};
}

/** Orientacao em odom de +X do corpo: angulo de proa no mundo. */
inline double headingWorld(const Quaternion &body_to_world)
{
    const Vector3 forward = rotate(body_to_world, {1.0, 0.0, 0.0});
    return std::atan2(forward.y, forward.x);
}

/** Velocidade de yaw no mundo, mesmo para o corpo invertido ~180 graus X. */
inline double yawRateWorld(const Quaternion &q, const Vector3 &omega_body)
{
    return rotate(q, omega_body).z;
}

inline double clamp(double value, double lower, double upper)
{
    return std::clamp(value, lower, upper);
}
}  // namespace ingenuity

#endif  // INGENUITY_NAVIGATION__FLIGHT_MATH_HPP_
