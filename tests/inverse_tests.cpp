#include <ping_pong/inverse.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}
double distance(ping_pong::Vec3 left, ping_pong::Vec3 right) {
    return std::hypot(left.x - right.x, left.y - right.y, left.z - right.z);
}
template<class Exception, class Function>
void check_throws(Function function) {
    try { function(); }
    catch (const Exception&) { check(true, "expected exception"); return; }
    check(false, "missing exception");
}
}

int main() {
    using namespace ping_pong;
    const auto gravity = [](const State&, double) { return Vec3{0, 0, -9.81}; };
    const Vec3 start{0, 0, 1};
    const Vec3 known_velocity{2, 1, 3};
    const State target = advance({start, known_velocity}, {0, 0, -9.81},
                                 std::chrono::duration<double>{0.5});

    // Independent constant-acceleration solution validates repeated RK4 steps.
    State stepped{start, known_velocity};
    for (int step = 0; step < 50; ++step) {
        stepped = integrate_step(stepped, step * 0.01, 0.01, gravity);
    }
    check(distance(stepped.position, target.position) < 1e-12, "stepped gravity position");
    check(distance(stepped.velocity, target.velocity) < 1e-12, "stepped gravity velocity");

    InverseOptions options;
    options.min_time = 0.2;
    options.max_time = 0.8;
    const auto solved = solve_inverse(start, target, {}, gravity, options);
    check(solved.best.position_error_norm < 0.002, "free-time solve position error");
    check(solved.best.velocity_error_norm < 0.02, "free-time solve velocity error");
    check(solved.search_step_converged, "search step converges");
    check(solved.trials > 1, "solver evaluates trial velocities");
    check(std::abs(solved.best.flight_time - 0.5) < 0.011, "finds interior arrival time");
    const State replay = advance({start, solved.best.initial_velocity}, {0, 0, -9.81},
                                  std::chrono::duration<double>{solved.best.flight_time});
    check(distance(replay.position, solved.best.arrival.position) < 1e-12,
          "reported arrival matches independent gravity reference");
    check(distance(replay.velocity, solved.best.arrival.velocity) < 1e-12,
          "reported arrival velocity matches independent gravity reference");
    const double expected_cost = std::pow(solved.best.position_error_norm / options.position_scale, 2)
        + std::pow(solved.best.velocity_error_norm / options.velocity_scale, 2);
    check(std::abs(expected_cost - solved.best.cost) < 1e-12, "reported cost matches errors");

    // Fixed, non-grid-aligned horizon must be sampled exactly.
    options.min_time = options.max_time = 0.503;
    const State off_grid_target = advance({start, known_velocity}, {0, 0, -9.81},
                                          std::chrono::duration<double>{0.503});
    const auto fixed = solve_inverse(start, off_grid_target, known_velocity, gravity, options);
    check(fixed.best.flight_time == 0.503, "exact fixed time boundary");
    check(fixed.best.position_error_norm < 1e-12, "fixed time target reached");

    // A velocity-dependent model uses the same solver. Independent linear-drag
    // solution: v(t)=v0*exp(-t), x(t)=x0+v0*(1-exp(-t)).
    std::size_t model_calls = 0;
    const auto drag = [&](const State& state, double) {
        ++model_calls;
        return Vec3{-state.velocity.x, -state.velocity.y, -state.velocity.z};
    };
    const double decay = std::exp(-0.5);
    const State drag_target{{2 * (1 - decay), 1 - decay, 1 + 3 * (1 - decay)},
                            {2 * decay, decay, 3 * decay}};
    options.min_time = options.max_time = 0.5;
    const auto drag_solution = solve_inverse(start, drag_target, {}, drag, options);
    check(drag_solution.best.position_error_norm < 0.001, "drag solve position error");
    check(drag_solution.best.velocity_error_norm < 0.001, "drag solve velocity error");
    check(model_calls > 4 * drag_solution.trials, "every trial takes multiple model steps");

    const auto drag_error = [&](double step_size) {
        State state{{}, {1, 0, 0}};
        const int steps = static_cast<int>(1 / step_size);
        for (int step = 0; step < steps; ++step) {
            state = integrate_step(state, step * step_size, step_size, drag);
        }
        return std::abs(state.velocity.x - std::exp(-1.0));
    };
    check(drag_error(0.1) < drag_error(0.2) / 10, "RK4 convergence with smaller steps");
    const auto time_dependent = [](const State&, double time) { return Vec3{time, 0, 0}; };
    const auto timed = integrate_step(State{}, 0, 1, time_dependent);
    check(std::abs(timed.position.x - 1.0 / 6) < 1e-14, "time-dependent stage position");
    check(std::abs(timed.velocity.x - 0.5) < 1e-14, "time-dependent stage velocity");

    options.max_launch_speed = 0;
    const auto stationary = solve_inverse(start, target, known_velocity, gravity, options);
    check(distance(stationary.best.initial_velocity, {}) == 0, "zero launch-speed limit");
    check(stationary.trials == 1, "zero speed has only one possible trial");
    check(stationary.best.position_error_norm > 0.1, "unreachable target reports residual");
    options.max_launch_speed = 1;
    const auto limited = solve_inverse(start, target, known_velocity, gravity, options);
    check(inverse_detail::norm(limited.best.initial_velocity) <= 1 + 1e-14, "speed ball enforced");
    options.max_launch_speed = 20;
    options.max_iterations = 1;
    const auto exhausted = solve_inverse(start, target, {}, gravity, options);
    check(!exhausted.search_step_converged, "budget exhaustion is not convergence");
    check(exhausted.trials == 7, "bounded iteration evaluates six neighbors");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const auto invalid_options = [&](InverseOptions invalid) {
        check_throws<std::invalid_argument>([&] {
            (void)solve_inverse(start, target, {}, gravity, invalid);
        });
    };
    for (double invalid : {0.0, -1.0, nan, infinity}) {
        auto changed = InverseOptions{}; changed.time_step = invalid; invalid_options(changed);
        changed = {}; changed.position_scale = invalid; invalid_options(changed);
        changed = {}; changed.velocity_scale = invalid; invalid_options(changed);
        changed = {}; changed.min_time = invalid; invalid_options(changed);
        changed = {}; changed.initial_velocity_step = invalid; invalid_options(changed);
        changed = {}; changed.velocity_step_tolerance = invalid; invalid_options(changed);
    }
    auto invalid = InverseOptions{}; invalid.max_time = 0.01; invalid_options(invalid);
    invalid = {}; invalid.max_launch_speed = -1; invalid_options(invalid);
    invalid = {}; invalid.max_iterations = 0; invalid_options(invalid);
    invalid = {}; invalid.time_step = 1e-10; invalid_options(invalid);
    check_throws<std::invalid_argument>([&] {
        (void)solve_inverse(start, target, {nan, 0, 0}, gravity);
    });
    check_throws<std::invalid_argument>([&] {
        (void)integrate_step(State{}, 0, -1, gravity);
    });
    const auto invalid_model = [&](const State&, double) { return Vec3{infinity, 0, 0}; };
    check_throws<std::overflow_error>([&] {
        (void)integrate_step(State{}, 0, 0.1, invalid_model);
    });
    check_throws<std::overflow_error>([&] {
        (void)solve_inverse(start, target, {}, invalid_model);
    });
    const auto throwing_model = [](const State&, double) -> Vec3 {
        throw std::runtime_error("model failure");
    };
    check_throws<std::runtime_error>([&] {
        (void)solve_inverse(start, target, {}, throwing_model);
    });
    std::cout << checks << " simulation/inverse checks passed\n";
}
