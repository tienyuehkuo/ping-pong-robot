# ping-pong-robot

Find a ball's initial velocity by trial and error, given its initial position,
target position, and desired velocity at that target.

## Steps

1. Choose a trial initial velocity.
2. Trace the ball in small time steps using the supplied net-force function.
3. Find the closest spatial passage to the target within the flight horizon.
4. Compare the velocity at that passage with the desired final velocity.
5. Try six neighboring initial velocities. Keep an improvement, or halve the
   velocity search step when none improves the result.

The implementation follows these same steps:

- `ball.hpp`: vectors, ball state, force interface, and a midpoint motion step.
- `trajectory.hpp`: stepping and closest passage to the target.
- `optimize.hpp`: trial evaluation and initial-velocity search.
- `examples/optimize_launch.cpp`: complete usage example with gravity.

## Force models

A `ForceModel` receives the current ball state and time since launch. It returns
**total force in newtons**, in the same coordinate frame as the position and
velocity. The integrator divides it by ball mass in kilograms. Combine any
supplied gravity, drag, or other force models in this function. It can depend
on position, velocity, and time; models must return deterministic values without
retaining changing simulation state between calls.

Each motion step uses the explicit midpoint method, evaluating force at the
start and midpoint. This is numerical integration, not a gravity-specific
inverse formula. Gravity in the example is only a placeholder for your models.

## Target passage and objective

The tracer selects the passage by **distance to target position alone**, not by
velocity error. It linearly interpolates position, velocity, and time along each
small step to estimate the nearest passage. Equal-distance ties retain the
earlier passage. The search covers time zero through `trace.max_time`.

A trial's cost is:

```text
(target miss distance / position tolerance)^2
  + (velocity error magnitude / velocity tolerance)^2
```

The tolerances normalize the errors and set their relative priority. The cost
is a compromise, but `target_reached` is true **only if both errors meet their
separate tolerances**. A missed target with matching velocity is not a success.

`search_converged` only says the search step reached `minimum_velocity_step`;
it does not imply the target was reached. The search can stop at the iteration
limit and return its best trial with `search_converged == false`.

This is a local search, not a guarantee of the global best launch velocity.
Initial guesses matter. Use smaller time steps to check integration and passage
accuracy. Midpoint stepping and segment interpolation assume smooth motion;
collisions and other discontinuities need separate handling. Flight time and
launch speed are bounded by the options. No physical aerodynamic model or
collision handling is included yet.

## Build and run

C++17, with no third-party dependencies. No tests are included.

```sh
cmake -S . -B build
cmake --build build
./build/optimize_launch
```

Or compile directly:

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude examples/optimize_launch.cpp -o /tmp/optimize_launch
/tmp/optimize_launch
```
