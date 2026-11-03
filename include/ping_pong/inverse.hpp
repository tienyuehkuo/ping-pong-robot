#ifndef PING_PONG_INVERSE_HPP
#define PING_PONG_INVERSE_HPP

#include <ping_pong/motion.hpp>
#include <algorithm>
#include <cstddef>
#include <vector>

namespace ping_pong {

struct InverseOptions {
    double min_time{0.05};             // seconds, strictly positive
    double max_time{3.0};              // seconds; equal bounds fix the time
    double max_launch_speed{20.0};     // m/s, including zero
    double position_scale{0.02};       // meters
    double velocity_scale{0.2};        // m/s
    std::size_t time_intervals{512};
    std::size_t refinement_iterations{64};
};

struct InverseResult {
    Vec3 initial_velocity;
    double flight_time;
    State arrival;
    Vec3 position_error;               // arrival minus target
    Vec3 velocity_error;
    double position_error_norm;
    double velocity_error_norm;
    double cost;                      // squared normalized errors, summed
};

namespace inverse_detail {
inline Vec3 subtract(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline double norm(Vec3 a) { return std::hypot(a.x, a.y, a.z); }
inline void validate(Vec3 start, const State& target, Vec3 acceleration,
                     const InverseOptions& o) {
    if (!is_finite(start) || !is_finite(target.position)
        || !is_finite(target.velocity) || !is_finite(acceleration)
        || !std::isfinite(o.min_time) || !std::isfinite(o.max_time)
        || o.min_time <= 0 || o.max_time < o.min_time
        || !std::isfinite(o.max_launch_speed) || o.max_launch_speed < 0
        || !std::isfinite(o.position_scale) || o.position_scale <= 0
        || !std::isfinite(o.velocity_scale) || o.velocity_scale <= 0
        || o.time_intervals < 2 || o.time_intervals > 1000000
        || o.refinement_iterations == 0 || o.refinement_iterations > 256) {
        throw std::invalid_argument("invalid inverse problem or search options");
    }
}

// For fixed T the objective is an isotropic quadratic in launch velocity.
// Its constrained optimum is its unconstrained optimum projected onto the
// speed ball. Long-double intermediates reduce avoidable scaling overflow.
inline InverseResult evaluate(Vec3 start, const State& target, Vec3 a,
                               const InverseOptions& o, double t) {
    const long double T = t;
    const long double sp = o.position_scale, sv = o.velocity_scale;
    const long double wp = 1 / (sp * sp), wv = 1 / (sv * sv);
    const auto component = [&](double p0, double p1, double v1, double acc) {
        const long double displacement = static_cast<long double>(p1) - p0;
        const long double position_term = displacement - 0.5L * acc * T * T;
        const long double velocity_term = static_cast<long double>(v1) - acc * T;
        const long double numerator = wp * T * position_term + wv * velocity_term;
        const long double denominator = wp * T * T + wv;
        return static_cast<double>(numerator / denominator);
    };
    Vec3 v{component(start.x, target.position.x, target.velocity.x, a.x),
           component(start.y, target.position.y, target.velocity.y, a.y),
           component(start.z, target.position.z, target.velocity.z, a.z)};
    const double speed = norm(v);
    if (!is_finite(v) || !std::isfinite(speed)) {
        throw std::overflow_error("inverse velocity exceeds numeric range");
    }
    if (speed > o.max_launch_speed) {
        const double factor = o.max_launch_speed / speed;
        v = {v.x * factor, v.y * factor, v.z * factor};
    }
    const State arrival = advance({start, v}, a, std::chrono::duration<double>{t});
    const Vec3 pe = subtract(arrival.position, target.position);
    const Vec3 ve = subtract(arrival.velocity, target.velocity);
    const double pn = norm(pe), vn = norm(ve);
    const double p = pn / o.position_scale, u = vn / o.velocity_scale;
    const double cost = p * p + u * u;
    if (!std::isfinite(cost)) {
        throw std::overflow_error("inverse cost exceeds numeric range");
    }
    return {v, t, arrival, pe, ve, pn, vn, cost};
}
} // namespace inverse_detail

// Constant acceleration only: no drag, spin, collisions, or path constraints.
// Position and velocity are matched at the same freely selected flight time.
// Bounded numerical search: samples a uniform time grid, then golden-section
// refines every sampled local-minimum bracket. Endpoints are always candidates.
// NOT a certified global optimizer; increase time_intervals to check stability.
// Finite but extreme inputs may throw overflow_error rather than yield a result.
[[nodiscard]] inline InverseResult solve_inverse(
    Vec3 start, const State& target,
    Vec3 acceleration = {0.0, 0.0, -9.81},
    const InverseOptions& options = {})
{
    using namespace inverse_detail;
    validate(start, target, acceleration, options);
    const auto evaluate_time = [&](double t) {
        return evaluate(start, target, acceleration, options, t);
    };
    InverseResult best = evaluate_time(options.min_time);
    if (options.min_time == options.max_time) return best;
    const auto consider = [&](const InverseResult& candidate) {
        if (candidate.cost < best.cost) best = candidate;
    };
    const auto time_at = [&](std::size_t i) {
        const double fraction = static_cast<double>(i) / options.time_intervals;
        return options.min_time + (options.max_time - options.min_time) * fraction;
    };
    std::vector<double> costs(options.time_intervals + 1);
    costs[0] = best.cost;
    for (std::size_t i = 1; i <= options.time_intervals; ++i) {
        const auto candidate = evaluate_time(time_at(i));
        costs[i] = candidate.cost;
        consider(candidate);
    }
    constexpr double ratio = 0.6180339887498948482;
    for (std::size_t i = 0; i <= options.time_intervals; ++i) {
        if ((i > 0 && costs[i] > costs[i-1])
            || (i < options.time_intervals && costs[i] > costs[i+1])) continue;
        double lo = time_at(i == 0 ? 0 : i-1);
        double hi = time_at(i == options.time_intervals ? i : i+1);
        auto left = evaluate_time(hi - ratio * (hi - lo));
        auto right = evaluate_time(lo + ratio * (hi - lo));
        consider(left);
        consider(right);
        for (std::size_t j = 0; j < options.refinement_iterations; ++j) {
            if (left.cost <= right.cost) {
                hi = right.flight_time;
                right = left;
                left = evaluate_time(hi - ratio * (hi - lo));
                consider(left);
            } else {
                lo = left.flight_time;
                left = right;
                right = evaluate_time(lo + ratio * (hi - lo));
                consider(right);
            }
        }
    }
    return best;
}

} // namespace ping_pong
#endif
