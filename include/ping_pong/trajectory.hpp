#pragma once

#include <ping_pong/ball.hpp>
#include <algorithm>

namespace ping_pong {

struct TraceOptions {
    double time_step{0.001}; // seconds
    double max_time{2.0};    // seconds since launch
};

struct TargetPassage {
    BallState ball;
    double time;
    double miss_distance;
};

// Locate the nearest point on one small trajectory segment. Interpolating
// between steps avoids missing the target merely because it lies between samples.
inline TargetPassage closest_passage_on_step(
    const BallState& start, const BallState& end, Vec3 target,
    double start_time, double time_step) {
    const Vec3 displacement = end.position - start.position;
    const double squared_distance = dot(displacement, displacement);
    if (!std::isfinite(squared_distance)) {
        throw std::runtime_error("trajectory segment exceeds numeric range");
    }
    double fraction = 0;
    if (squared_distance > 0) {
        fraction = dot(target - start.position, displacement) / squared_distance;
        if (!std::isfinite(fraction)) {
            throw std::runtime_error("target projection exceeds numeric range");
        }
        fraction = std::clamp(fraction, 0.0, 1.0);
    }
    const BallState passage{
        start.position + displacement * fraction,
        start.velocity + (end.velocity - start.velocity) * fraction
    };
    return {passage, start_time + fraction * time_step,
            length(passage.position - target)};
}

// Trace the whole bounded flight and retain its closest spatial passage.
// Desired final velocity does not affect which passage is selected.
inline TargetPassage trace_to_target(
    const BallState& initial, Vec3 target, double mass,
    const ForceModel& force, const TraceOptions& options = {}) {
    if (!is_finite(initial.position) || !is_finite(initial.velocity) || !is_finite(target)
        || !std::isfinite(mass) || mass <= 0
        || !std::isfinite(options.time_step) || options.time_step <= 0
        || !std::isfinite(options.max_time) || options.max_time <= 0) {
        throw std::invalid_argument("invalid trajectory input");
    }
    BallState ball = initial;
    double time = 0;
    TargetPassage closest{ball, time, length(ball.position - target)};
    while (time < options.max_time) {
        const double next_time = std::min(time + options.time_step, options.max_time);
        if (next_time <= time) throw std::runtime_error("time step cannot advance time");
        const double step_duration = next_time - time;
        const BallState next = step_ball(ball, time, step_duration, mass, force);
        const auto passage = closest_passage_on_step(ball, next, target, time, step_duration);
        if (passage.miss_distance < closest.miss_distance) closest = passage;
        ball = next;
        time = next_time;
    }
    return closest;
}

} // namespace ping_pong
