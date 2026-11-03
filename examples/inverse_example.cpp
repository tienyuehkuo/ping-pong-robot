#include <ping_pong/inverse.hpp>
#include <iostream>

int main() {
    const ping_pong::Vec3 start{0, 0, 1};
    const ping_pong::State target{{2, 0, 0.8}, {4, 0, -3}};
    ping_pong::InverseOptions options;
    options.min_time = 0.1;
    options.max_time = 1.5;
    options.max_launch_speed = 10;
    const auto r = ping_pong::solve_inverse(start, target, {0, 0, -9.81}, options);
    std::cout << "Launch velocity (m/s): " << r.initial_velocity.x << ", "
              << r.initial_velocity.y << ", " << r.initial_velocity.z
              << "\nFlight duration (s): " << r.flight_time
              << "\nPosition error (m): " << r.position_error_norm
              << "\nVelocity error (m/s): " << r.velocity_error_norm
              << "\nNormalized cost: " << r.cost << '\n';
}
