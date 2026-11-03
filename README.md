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

## Gravity-only inverse trajectory optimization

`include/ping_pong/inverse.hpp` chooses initial velocity **and flight duration**
to approximately match a desired final position and velocity at the same time.
Initial absolute time only shifts the arrival timestamp: `arrival_time = t0 + T`.
All vectors must share one inertial coordinate frame. Acceleration is constant;
the default is gravity `(0, 0, -9.81)` m/s².

```cpp
#include <ping_pong/inverse.hpp>

ping_pong::InverseOptions options;
options.min_time = 0.1;           // seconds
options.max_time = 1.5;
options.max_launch_speed = 10;   // m/s; spherical speed constraint
options.position_scale = 0.02;  // meters
options.velocity_scale = 0.2;   // m/s
const auto result = ping_pong::solve_inverse(
    {0, 0, 1}, {{2, 0, 0.8}, {4, 0, -3}}, {0, 0, -9.81}, options);
```

The objective is `J = (position_error_norm / position_scale)^2 +
(velocity_error_norm / velocity_scale)^2`. Scales normalize units and determine
relative importance; they are **not hard tolerances**. This is a weighted
compromise, not a guarantee of hitting the target region. Inspect both reported
errors before accepting a trajectory. Results include signed error vectors,
error norms in m and m/s, arrival state, launch velocity, duration, and cost.

At each T the objective is an isotropic quadratic in launch velocity. Writing
`d = p_target - p0 - 0.5*a*T*T`, `b = v_target - a*T`,
`wp = 1/position_scale²`, and `wv = 1/velocity_scale²`, its exact unconstrained
minimizer is `(wp*T*d + wv*b)/(wp*T*T + wv)`. Projecting this vector onto the
launch-speed ball gives the constrained fixed-time optimum.

The remaining bounded time search samples 512 intervals by default and
refines every sampled local-minimum bracket using 64 golden-section steps.
Both endpoints remain candidates. **This is a numerical approximation, not a
certified global minimum**: narrow or unresolved minima may be missed. Increase
`time_intervals` and compare results to assess search stability. Equal positive
time bounds allow a fixed-time solve. All inputs must be finite; scales and
time must be positive; speed can be zero. Invalid options throw
`std::invalid_argument`; extreme arithmetic that cannot be represented throws
`std::overflow_error`.

Build with CMake as above, then run `./build/inverse_example`. Without CMake:

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude tests/inverse_tests.cpp -o /tmp/inverse_tests
/tmp/inverse_tests
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude examples/inverse_example.cpp -o /tmp/inverse_example
/tmp/inverse_example
```

This milestone does **not** include aerodynamics, spin, bounces, net/table
clearance, racket kinematics, or collision avoidance. The speed bound alone does
not establish robot feasibility. Planned progression: validate this baseline,
introduce numerical forward integration with drag, then spin/Magnus effects
and any required collisions. No third-party optimizer is needed for this
analytically reducible baseline; reassess maintained optimization libraries for
the aerodynamic model.
