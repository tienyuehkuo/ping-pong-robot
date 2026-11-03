#include <ping_pong/inverse.hpp>
#include <iostream>

int main() {
    using namespace ping_pong;
    const Vec3 start{0, 0, 1};
    const State target{{1, 0.5, 1.27375}, {2, 1, -1.905}};
    const auto gravity = [](const State&, double) { return Vec3{0, 0, -9.81}; };
    InverseOptions options;
    options.max_time = 1;
    const auto result = solve_inverse(start, target, Vec3{}, gravity, options);
    const auto& best = result.best;
    std::cout << "Launch velocity (m/s): " << best.initial_velocity.x << ", "
              << best.initial_velocity.y << ", " << best.initial_velocity.z
              << "\nFlight time (s): " << best.flight_time
              << "\nPosition error (m): " << best.position_error_norm
              << "\nVelocity error (m/s): " << best.velocity_error_norm
              << "\nTrials: " << result.trials
              << "\nSearch step converged: " << std::boolalpha << result.search_step_converged
              << '\n';
}
