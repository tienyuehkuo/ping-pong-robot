#include <ping_pong/optimize.hpp>
#include <iostream>

int main() {
    using namespace ping_pong;
    const LaunchProblem problem{
        {0, 0, 1},                    // initial position, meters
        {1, 0.5, 1.27375},           // target position, meters
        {2, 1, -1.905},              // desired velocity at the target, m/s
        0.0027                       // ball mass, kg
    };
    // Example only: replace this function with the supplied net-force models.
    const ForceModel gravity = [mass = problem.mass](const BallState&, double) {
        return Vec3{0, 0, -mass * 9.81};
    };
    SearchOptions options;
    options.trace.max_time = 1;
    const Vec3 initial_guess{1, 0, 1};
    const auto result = optimize_launch(problem, initial_guess, gravity, options);

    std::cout << "Initial velocity (m/s): " << result.initial_velocity.x << ", "
              << result.initial_velocity.y << ", " << result.initial_velocity.z
              << "\nClosest passage time (s): " << result.passage.time
              << "\nTarget miss (m): " << result.passage.miss_distance
              << "\nVelocity error (m/s): " << length(result.velocity_error)
              << "\nBoth target tolerances met: " << std::boolalpha << result.target_reached
              << "\nSearch step converged: " << result.search_converged << '\n';
}
