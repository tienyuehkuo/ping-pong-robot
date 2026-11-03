# ping-pong-robot

## 3D motion under constant acceleration

A dependency-free, header-only C++17 function in
`include/ping_pong/motion.hpp` computes a point's next position and velocity:

```cpp
#include <ping_pong/motion.hpp>
#include <chrono>

const ping_pong::State initial{{0.0, 0.0, 1.0}, {2.0, 0.0, 3.0}};
const ping_pong::Vec3 gravity{0.0, 0.0, -9.81}; // z points up
const auto next = ping_pong::advance(
    initial, gravity, std::chrono::milliseconds{10});
```

The equations are applied independently to x, y, and z:

```text
position_next = position + velocity * dt + 0.5 * acceleration * dt^2
velocity_next = velocity + acceleration * dt
```

Use meters, seconds, m/s, and m/s^2 in a single coordinate frame. Acceleration
must remain constant over the entire step. Drag, spin, collisions, and table
bounces are not modeled. Results are subject to floating-point rounding.

`advance` returns a new `State`; it does not modify its inputs or allocate
dynamic memory on the successful path. All coordinates default to zero.
It accepts `std::chrono::duration<double>` (seconds); integral milliseconds
and other safely convertible chrono durations can be passed directly.

- Negative time or any NaN/infinite input throws `std::invalid_argument`.
- Zero time returns the initial state after input validation.
- A non-finite calculated result throws `std::overflow_error`.

This is checked floating-point arithmetic, not a guarantee against precision
loss, underflow, or every avoidable overflow for extreme magnitudes. Do not
compile with `-ffast-math`, which can invalidate finite-value checks.

## Build and test

Requires a C++17 compiler and CMake 3.14 or newer; no third-party libraries.

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/motion_example
```

Or build directly with a C++17 compiler:

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude tests/motion_tests.cpp -o /tmp/motion_tests
/tmp/motion_tests
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude examples/motion_example.cpp -o /tmp/motion_example
/tmp/motion_example
```

## Simulation-based inverse trajectory search

`include/ping_pong/inverse.hpp` searches for a launch velocity that brings a
ball near a target **position and velocity at the same arrival time**:

1. Choose a trial launch velocity, bounded by `max_launch_speed`.
2. Simulate from the starting position through small time steps.
3. Score arrival samples between `min_time` and `max_time`. Refine the best
   sample's neighboring time interval with partial forward-integration steps.
4. Compare the trial with six neighboring velocities (positive/negative x, y,
   and z). Move to an improvement, or halve the velocity search step if none
   improves the cost. Repeat until the search step is small or the budget ends.

Every trial runs the forward simulation. There is no analytic gravity-only
inverse formula. The header `trajectory.hpp` supplies a fourth-order
Runge-Kutta step (`integrate_step`). It calls an acceleration model with
`(const State&, elapsed_seconds)` at each RK4 stage. Gravity is the first model;
a later model can add aerodynamic acceleration based on the ball's current
state without changing the search algorithm. The existing constant-acceleration
`advance` function is unchanged.

```cpp
#include <ping_pong/inverse.hpp>

const ping_pong::Vec3 start{0, 0, 1};
const ping_pong::State target{{1, 0.5, 1.27375}, {2, 1, -1.905}};
const auto gravity = [](const ping_pong::State&, double) {
    return ping_pong::Vec3{0, 0, -9.81};
};
ping_pong::InverseOptions options;
options.max_time = 1.0;
options.time_step = 0.01;
const auto result = ping_pong::solve_inverse(
    start, target, ping_pong::Vec3{}, gravity, options);
// Inspect result.best.position_error_norm and velocity_error_norm before use.
```

### Error metric and limits

The cost is the sum of squared normalized errors:

```text
(position_error_norm / position_scale)^2
  + (velocity_error_norm / velocity_scale)^2
```

Scales set the relative priorities; they are **not hard tolerances**. Errors
are arrival minus target. Results report the launch velocity, arrival state,
arrival time, both error vectors/norms, cost, number of simulated trials, and
whether the velocity search step reached its stopping tolerance.

This is a **local numerical search**, not a certified global optimizer or a
feasibility guarantee. Initial guesses matter. `search_step_converged` means
only that no tested neighboring move improved the result at the specified
velocity resolution. An unreachable target can still have a converged search
with large errors. `false` means the iteration budget ended first. With zero
maximum launch speed there is only one possible trial.

The time grid and its local refinement can miss another, better arrival
interval. Smaller `time_step` values improve integration/sampling resolution,
but cost more. Repeat with smaller time steps and different velocity guesses
and compare **both** reported errors before accepting a result. RK4 assumes a
smooth acceleration model; it is not an event/collision handler. Each refined
arrival is integrated from a saved state near that arrival, not interpolated
from endpoint coordinates.

All input vectors and options must be finite. Time bounds, time step, scales,
and search step sizes must be positive; time bounds may be equal. The speed
limit may be zero. The stopping tolerance cannot exceed the initial search
step. For bounded work, the horizon/time-step ratio is at most 100,000 and the
iteration budget is 1–10,000. Each iteration evaluates six trial velocities;
each trial integrates the horizon and performs up to 34 local time-refinement
samples. Invalid inputs throw `std::invalid_argument`. Non-finite integration
results, non-finite model outputs, non-advancing time steps, and non-finite
costs throw `std::overflow_error` rather than silently reporting success.
Model exceptions propagate; models must return deterministic accelerations in
m/s² using the same coordinate frame. Model callbacks must not retain changing
physical state between calls. Floating-point rounding/underflow remain possible;
do not use `-ffast-math`.

No physical drag/spin model, table/net collision, racket motion, or trajectory
clearance constraint is implemented. The tests use simple linear drag with an
independent analytic reference only to verify that the interface supports
velocity-dependent acceleration.

CMake builds `inverse_tests` and `inverse_example` along with the existing
motion targets. Or compile directly:

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude tests/inverse_tests.cpp -o /tmp/inverse_tests
/tmp/inverse_tests
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude examples/inverse_example.cpp -o /tmp/inverse_example
/tmp/inverse_example
```

This small, dependency-free baseline uses explicit RK4 and coordinate-search
steps to keep the requested trial/simulate/score loop visible. Reassess a
maintained numerical library if adaptive integration or more advanced
optimization becomes necessary.
