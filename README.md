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
