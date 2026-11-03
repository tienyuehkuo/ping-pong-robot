#pragma once

#include <ping_pong/trajectory.hpp>
#include <cstddef>

namespace ping_pong {

struct LaunchProblem {
    Vec3 initial_position;
    Vec3 target_position;
    Vec3 target_velocity;
    double mass{0.0027}; // kilograms
};

struct SearchOptions {
    TraceOptions trace;
    double max_launch_speed{20.0};       // meters per second
    double initial_velocity_step{1.0};   // meters per second
    double minimum_velocity_step{0.001}; // meters per second
    double position_tolerance{0.01};     // meters
    double velocity_tolerance{0.1};      // meters per second
    std::size_t iteration_limit{200};
};

struct LaunchResult {
    Vec3 initial_velocity;
    TargetPassage passage;
    Vec3 velocity_error; // passage velocity minus desired target velocity
    double cost;
    bool target_reached;
    bool search_converged{false};
};

inline Vec3 limit_launch_speed(Vec3 velocity, double maximum_speed) {
    const double speed = length(velocity);
    if (!is_finite(velocity) || !std::isfinite(speed)
        || !std::isfinite(maximum_speed) || maximum_speed < 0) {
        throw std::invalid_argument("invalid launch velocity or speed limit");
    }
    if (speed > maximum_speed) velocity = velocity * (maximum_speed / speed);
    return velocity;
}

// Evaluate one trial: trace first, then compare velocity at the closest passage.
inline LaunchResult evaluate_launch(const LaunchProblem& problem, Vec3 velocity,
                                    const ForceModel& force, const SearchOptions& options) {
    if (!is_finite(problem.target_velocity)
        || !std::isfinite(options.position_tolerance) || options.position_tolerance <= 0
        || !std::isfinite(options.velocity_tolerance) || options.velocity_tolerance <= 0) {
        throw std::invalid_argument("invalid target velocity or error tolerances");
    }
    velocity = limit_launch_speed(velocity, options.max_launch_speed);
    const auto passage = trace_to_target(
        {problem.initial_position, velocity}, problem.target_position,
        problem.mass, force, options.trace);
    const Vec3 velocity_error = passage.ball.velocity - problem.target_velocity;
    const double velocity_error_size = length(velocity_error);
    const double position_cost = passage.miss_distance / options.position_tolerance;
    const double velocity_cost = velocity_error_size / options.velocity_tolerance;
    const double cost = position_cost * position_cost + velocity_cost * velocity_cost;
    if (!std::isfinite(cost)) throw std::runtime_error("non-finite trajectory cost");
    const bool reached = passage.miss_distance <= options.position_tolerance
        && velocity_error_size <= options.velocity_tolerance;
    return {velocity, passage, velocity_error, cost, reached};
}

// Local trial-and-error search. Poll six neighbors, move to an improvement,
// or reduce the velocity step when none improves the result.
inline LaunchResult optimize_launch(
    const LaunchProblem& problem, Vec3 initial_guess,
    const ForceModel& force, const SearchOptions& options = {}) {
    if (!std::isfinite(options.initial_velocity_step) || options.initial_velocity_step <= 0
        || !std::isfinite(options.minimum_velocity_step) || options.minimum_velocity_step <= 0
        || options.minimum_velocity_step > options.initial_velocity_step
        || options.iteration_limit == 0) {
        throw std::invalid_argument("invalid velocity-search settings");
    }
    const Vec3 directions[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    LaunchResult best = evaluate_launch(problem, initial_guess, force, options);
    double velocity_step = options.initial_velocity_step;
    for (std::size_t iteration = 0;
         iteration < options.iteration_limit && velocity_step > options.minimum_velocity_step;
         ++iteration) {
        const Vec3 center = best.initial_velocity;
        const double previous_cost = best.cost;
        for (Vec3 direction : directions) {
            const Vec3 candidate = center + direction * velocity_step;
            const auto trial = evaluate_launch(problem, candidate, force, options);
            if (trial.cost < best.cost) best = trial;
        }
        if (best.cost >= previous_cost) velocity_step /= 2;
    }
    best.search_converged = velocity_step <= options.minimum_velocity_step;
    return best;
}

} // namespace ping_pong
