#ifndef PING_PONG_TRAJECTORY_HPP
#define PING_PONG_TRAJECTORY_HPP

#include <ping_pong/motion.hpp>

namespace ping_pong {
namespace trajectory_detail {
inline Vec3 add_scaled(Vec3 base, Vec3 change, double scale) {
    return {base.x + change.x * scale, base.y + change.y * scale,
            base.z + change.z * scale};
}
inline State offset(State state, State derivative, double step) {
    return {add_scaled(state.position, derivative.position, step),
            add_scaled(state.velocity, derivative.velocity, step)};
}
inline void require_finite(const State& state) {
    if (!is_finite(state.position) || !is_finite(state.velocity)) {
        throw std::overflow_error("trajectory exceeds finite numeric range");
    }
}
} // namespace trajectory_detail

// One fourth-order Runge-Kutta step. The model returns acceleration in m/s^2
// from (state, elapsed seconds). It is re-evaluated at each RK4 stage, so later
// drag/spin models can depend on velocity, position, and time. No collisions.
// Models must be deterministic; exceptions propagate to the caller.
template<class AccelerationModel>
[[nodiscard]] State integrate_step(const State& initial, double elapsed_time,
                                   double time_step, AccelerationModel&& acceleration) {
    using namespace trajectory_detail;
    if (!is_finite(initial.position) || !is_finite(initial.velocity)
        || !std::isfinite(elapsed_time) || elapsed_time < 0
        || !std::isfinite(time_step) || time_step < 0) {
        throw std::invalid_argument("invalid numerical integration input");
    }
    if (!std::isfinite(elapsed_time + time_step)) {
        throw std::overflow_error("trajectory time exceeds finite numeric range");
    }
    if (time_step == 0) return initial;
    const auto derivative = [&](const State& state, double time) {
        require_finite(state);
        const State slope{state.velocity, acceleration(state, time)};
        require_finite(slope);
        return slope;
    };
    const State first = derivative(initial, elapsed_time);
    const State second = derivative(offset(initial, first, time_step / 2),
                                    elapsed_time + time_step / 2);
    const State third = derivative(offset(initial, second, time_step / 2),
                                   elapsed_time + time_step / 2);
    const State fourth = derivative(offset(initial, third, time_step),
                                    elapsed_time + time_step);
    // Accumulate the RK4 weighted average without adding unscaled slopes first.
    State next = offset(initial, first, time_step / 6);
    next = offset(next, second, time_step / 3);
    next = offset(next, third, time_step / 3);
    next = offset(next, fourth, time_step / 6);
    require_finite(next);
    return next;
}

} // namespace ping_pong
#endif
