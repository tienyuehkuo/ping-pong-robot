#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>

namespace ping_pong {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

inline Vec3 operator+(Vec3 left, Vec3 right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 operator-(Vec3 left, Vec3 right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 operator*(Vec3 vector, double scale) {
    return {vector.x * scale, vector.y * scale, vector.z * scale};
}

inline double dot(Vec3 left, Vec3 right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline double length(Vec3 vector) {
    return std::hypot(vector.x, vector.y, vector.z);
}

inline bool is_finite(Vec3 vector) {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

struct BallState {
    Vec3 position; // meters
    Vec3 velocity; // meters per second
};

// Return the total force in newtons. Time is seconds since launch.
// Models may use position, velocity, and time, but must not mutate the ball.
using ForceModel = std::function<Vec3(const BallState& ball, double time)>;

inline Vec3 acceleration(const BallState& ball, double time,
                         double mass, const ForceModel& force) {
    const Vec3 value = force(ball, time) * (1.0 / mass);
    if (!is_finite(value)) throw std::runtime_error("non-finite acceleration");
    return value;
}

// Explicit midpoint integration: evaluate forces at the start and midpoint.
inline BallState step_ball(const BallState& ball, double time, double time_step,
                           double mass, const ForceModel& force) {
    if (!is_finite(ball.position) || !is_finite(ball.velocity)
        || !std::isfinite(time) || time < 0
        || !std::isfinite(time_step) || time_step <= 0
        || !std::isfinite(mass) || mass <= 0) {
        throw std::invalid_argument("invalid ball, time step, or mass");
    }
    const double half_step = time_step / 2;
    const Vec3 initial_acceleration = acceleration(ball, time, mass, force);
    const BallState midpoint{
        ball.position + ball.velocity * half_step,
        ball.velocity + initial_acceleration * half_step
    };
    if (!is_finite(midpoint.position) || !is_finite(midpoint.velocity)
        || !std::isfinite(time + time_step)) {
        throw std::runtime_error("non-finite integration midpoint or time");
    }
    const Vec3 midpoint_acceleration = acceleration(midpoint, time + half_step, mass, force);
    const BallState next{
        ball.position + midpoint.velocity * time_step,
        ball.velocity + midpoint_acceleration * time_step
    };
    if (!is_finite(next.position) || !is_finite(next.velocity)) {
        throw std::runtime_error("non-finite trajectory state");
    }
    return next;
}

} // namespace ping_pong
