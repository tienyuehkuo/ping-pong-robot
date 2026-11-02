#ifndef PING_PONG_MOTION_HPP
#define PING_PONG_MOTION_HPP

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ping_pong {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

struct State {
    Vec3 position{};
    Vec3 velocity{};
};

[[nodiscard]] inline bool is_finite(Vec3 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

// Constant acceleration throughout dt; all vectors use the same coordinate frame.
// Units: position in meters, velocity in m/s, acceleration in m/s^2.
// Throws invalid_argument for non-finite inputs or negative time;
// throws overflow_error if a calculated component is not finite.
[[nodiscard]] inline State advance(
    const State& initial,
    Vec3 acceleration,
    std::chrono::duration<double> dt)
{
    const double t = dt.count();
    if (!is_finite(initial.position) || !is_finite(initial.velocity)
        || !is_finite(acceleration) || !std::isfinite(t) || t < 0.0) {
        throw std::invalid_argument("advance requires finite inputs and dt >= 0");
    }
    if (t == 0.0) {
        return initial;
    }

    const auto velocity = [t](double v, double a) {
        return std::fma(a, t, v);
    };
    const auto position = [t](double p, double v, double a) {
        // Avoid forming t*t, which could overflow even when a is zero.
        return std::fma(std::fma(0.5 * a, t, v), t, p);
    };

    const State next{
        {position(initial.position.x, initial.velocity.x, acceleration.x),
         position(initial.position.y, initial.velocity.y, acceleration.y),
         position(initial.position.z, initial.velocity.z, acceleration.z)},
        {velocity(initial.velocity.x, acceleration.x),
         velocity(initial.velocity.y, acceleration.y),
         velocity(initial.velocity.z, acceleration.z)}
    };
    if (!is_finite(next.position) || !is_finite(next.velocity)) {
        throw std::overflow_error("advance result exceeds finite double range");
    }
    return next;
}

} // namespace ping_pong

#endif // PING_PONG_MOTION_HPP
