#ifndef PING_PONG_INVERSE_HPP
#define PING_PONG_INVERSE_HPP

#include <ping_pong/trajectory.hpp>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <initializer_list>

namespace ping_pong {

struct InverseOptions {
    double min_time{0.05};               // earliest arrival sample, seconds
    double max_time{3.0};                // simulation horizon, seconds
    double time_step{0.01};              // integration/sample interval, seconds
    double max_launch_speed{20.0};       // m/s
    double position_scale{0.02};         // meters; error normalization, not tolerance
    double velocity_scale{0.2};          // m/s; error normalization, not tolerance
    double initial_velocity_step{2.0};   // initial search displacement, m/s
    double velocity_step_tolerance{1e-4}; // stop when search step reaches this, m/s
    std::size_t max_iterations{256};
};

struct LaunchTrial {
    Vec3 initial_velocity;
    double flight_time;
    State arrival;
    Vec3 position_error;                // arrival minus target
    Vec3 velocity_error;
    double position_error_norm;
    double velocity_error_norm;
    double cost;                        // sum of squared normalized errors
};

struct InverseResult {
    LaunchTrial best;
    std::size_t trials;
    bool search_step_converged;         // NOT a feasibility or global-optimum claim
};

namespace inverse_detail {
inline double norm(Vec3 vector) {
    return std::hypot(vector.x, vector.y, vector.z);
}
inline Vec3 difference(Vec3 left, Vec3 right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}
inline void validate(Vec3 start, const State& target, Vec3 guess,
                     const InverseOptions& options) {
    if (!is_finite(start) || !is_finite(target.position)
        || !is_finite(target.velocity) || !is_finite(guess)
        || !std::isfinite(options.min_time) || options.min_time <= 0
        || !std::isfinite(options.max_time) || options.max_time < options.min_time
        || !std::isfinite(options.time_step) || options.time_step <= 0
        || options.max_time / options.time_step > 100000
        || !std::isfinite(options.max_launch_speed) || options.max_launch_speed < 0
        || !std::isfinite(options.position_scale) || options.position_scale <= 0
        || !std::isfinite(options.velocity_scale) || options.velocity_scale <= 0
        || !std::isfinite(options.initial_velocity_step) || options.initial_velocity_step <= 0
        || !std::isfinite(options.velocity_step_tolerance) || options.velocity_step_tolerance <= 0
        || options.velocity_step_tolerance > options.initial_velocity_step
        || options.max_iterations == 0 || options.max_iterations > 10000) {
        throw std::invalid_argument("invalid inverse problem or search options");
    }
}
inline Vec3 limit_speed(Vec3 velocity, double maximum_speed) {
    const double speed = norm(velocity);
    if (!is_finite(velocity) || !std::isfinite(speed)) {
        throw std::overflow_error("launch velocity exceeds finite numeric range");
    }
    if (speed <= maximum_speed) return velocity;
    const double factor = maximum_speed / speed;
    return {velocity.x * factor, velocity.y * factor, velocity.z * factor};
}
inline LaunchTrial score(Vec3 launch_velocity, double time,
                          const State& arrival, const State& target,
                          const InverseOptions& options) {
    const Vec3 position_error = difference(arrival.position, target.position);
    const Vec3 velocity_error = difference(arrival.velocity, target.velocity);
    const double position_error_norm = norm(position_error);
    const double velocity_error_norm = norm(velocity_error);
    const double normalized_position_error = position_error_norm / options.position_scale;
    const double normalized_velocity_error = velocity_error_norm / options.velocity_scale;
    const double cost = normalized_position_error * normalized_position_error
        + normalized_velocity_error * normalized_velocity_error;
    if (!std::isfinite(cost)) {
        throw std::overflow_error("trajectory error exceeds finite numeric range");
    }
    return {launch_velocity, time, arrival, position_error, velocity_error,
            position_error_norm, velocity_error_norm, cost};
}

// Refine an arrival bracket by integrating from its saved starting state.
// No gravity equations or inverse formulas are used here.
template<class AccelerationModel>
LaunchTrial refine_arrival_time(
    LaunchTrial best, const State& refinement_start, double refinement_start_time,
    double refinement_end_time, const State& target,
    AccelerationModel& acceleration, const InverseOptions& options) {
    const Vec3 launch_velocity = best.initial_velocity;
    // Refine the best sampled arrival's neighboring interval by forward
    // integrating partial steps. This avoids locking the velocity search to
    // a coarse arrival-time grid. It is still only a local time refinement.
    double lower_time = std::max(refinement_start_time, options.min_time);
    double upper_time = refinement_end_time;
    const auto sample_time = [&](double time) {
        const double first_step = std::min(time - refinement_start_time, options.time_step);
        State arrival = integrate_step(refinement_start, refinement_start_time,
                                       first_step, acceleration);
        const double remaining_step = time - refinement_start_time - first_step;
        if (remaining_step > 0) {
            arrival = integrate_step(arrival, refinement_start_time + first_step,
                                     remaining_step, acceleration);
        }
        const auto sample = score(launch_velocity, time, arrival, target, options);
        if (sample.cost < best.cost) best = sample;
        return sample.cost;
    };
    constexpr double golden_ratio = 0.6180339887498948482;
    double left_time = upper_time - golden_ratio * (upper_time - lower_time);
    double right_time = lower_time + golden_ratio * (upper_time - lower_time);
    double left_cost = sample_time(left_time);
    double right_cost = sample_time(right_time);
    for (int iteration = 0; iteration < 32 && lower_time < upper_time; ++iteration) {
        if (left_cost <= right_cost) {
            upper_time = right_time;
            right_time = left_time;
            right_cost = left_cost;
            left_time = upper_time - golden_ratio * (upper_time - lower_time);
            left_cost = sample_time(left_time);
        } else {
            lower_time = left_time;
            left_time = right_time;
            left_cost = right_cost;
            right_time = lower_time + golden_ratio * (upper_time - lower_time);
            right_cost = sample_time(right_time);
        }
    }
    return best;
}

// A trial always simulates from the launch point. Keep the best arrival sample
// near the target in the weighted position/velocity metric, not just the final
// state at the horizon. Include both time bounds exactly; no time-zero shortcut.
template<class AccelerationModel>
LaunchTrial simulate_trial(Vec3 start, Vec3 launch_velocity, const State& target,
                           AccelerationModel& acceleration, const InverseOptions& options) {
    State state{start, launch_velocity};
    double elapsed_time = 0;
    State refinement_start = state;
    double refinement_start_time = 0;
    double refinement_end_time = options.min_time;
    LaunchTrial best{};
    best.cost = std::numeric_limits<double>::infinity();
    while (elapsed_time < options.max_time) {
        const double next_boundary = elapsed_time < options.min_time
            ? options.min_time : options.max_time;
        const double next_time = std::min(elapsed_time + options.time_step, next_boundary);
        if (next_time <= elapsed_time) {
            throw std::overflow_error("time step cannot advance trajectory time");
        }
        const State previous_state = state;
        const double previous_time = elapsed_time;
        state = integrate_step(state, elapsed_time, next_time - elapsed_time, acceleration);
        elapsed_time = next_time;
        if (elapsed_time >= options.min_time) {
            const auto sample = score(launch_velocity, elapsed_time, state, target, options);
            if (sample.cost < best.cost) {
                best = sample;
                refinement_start = previous_state;
                refinement_start_time = previous_time;
                refinement_end_time = std::min(elapsed_time + options.time_step,
                                               options.max_time);
            }
        }
    }
    return refine_arrival_time(best, refinement_start, refinement_start_time,
                               refinement_end_time, target, acceleration, options);
}
} // namespace inverse_detail

// Derivative-free coordinate search over launch velocity. Every candidate is
// scored by numerical forward simulation; there is no gravity-specific inverse
// formula. Supply a deterministic acceleration(state, elapsed_seconds) model.
// Local, bounded search only: inspect errors and repeat with other guesses and
// smaller time steps. Callback exceptions and numeric overflow propagate.
template<class AccelerationModel>
[[nodiscard]] InverseResult solve_inverse(
    Vec3 start, const State& target, Vec3 initial_velocity_guess,
    AccelerationModel&& acceleration, const InverseOptions& options = {}) {
    using namespace inverse_detail;
    validate(start, target, initial_velocity_guess, options);
    const auto evaluate = [&](Vec3 velocity) {
        return simulate_trial(start, limit_speed(velocity, options.max_launch_speed),
                              target, acceleration, options);
    };
    LaunchTrial best = evaluate(initial_velocity_guess);
    std::size_t trials = 1;
    if (options.max_launch_speed == 0) return {best, trials, true};
    double velocity_step = options.initial_velocity_step;
    for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
        if (velocity_step <= options.velocity_step_tolerance) return {best, trials, true};
        // Poll all six neighbors of the same center before choosing a move.
        const Vec3 center = best.initial_velocity;
        const double previous_cost = best.cost;
        for (int axis = 0; axis < 3; ++axis) {
            for (double direction : {-1.0, 1.0}) {
                Vec3 candidate = center;
                if (axis == 0) candidate.x += direction * velocity_step;
                if (axis == 1) candidate.y += direction * velocity_step;
                if (axis == 2) candidate.z += direction * velocity_step;
                const auto trial = evaluate(candidate);
                ++trials;
                if (trial.cost < best.cost) best = trial;
            }
        }
        if (best.cost >= previous_cost) velocity_step /= 2;
    }
    return {best, trials, velocity_step <= options.velocity_step_tolerance};
}

} // namespace ping_pong
#endif
