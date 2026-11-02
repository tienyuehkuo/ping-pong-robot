#include <ping_pong/motion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int checks = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool close(double actual, double expected) {
    return std::abs(actual - expected)
        <= 1e-12 * std::max(1.0, std::abs(expected));
}

void check_vector(ping_pong::Vec3 actual, ping_pong::Vec3 expected) {
    check(close(actual.x, expected.x) && close(actual.y, expected.y)
              && close(actual.z, expected.z), "vector matches expected value");
}

template<typename Exception, typename Function>
void check_throws(Function function) {
    try {
        function();
    } catch (const Exception&) {
        check(true, "expected exception");
        return;
    }
    check(false, "expected exception was not thrown");
}
} // namespace

int main() {
    using ping_pong::advance;
    using ping_pong::State;
    using ping_pong::Vec3;
    using Seconds = std::chrono::duration<double>;
    const State initial{{1.0, -2.0, 3.0}, {4.0, -5.0, 6.0}};
    const Vec3 acceleration{-2.0, 3.0, -4.0};

    const auto next = advance(initial, acceleration, Seconds{2.0});
    check_vector(next.position, {5.0, -6.0, 7.0});
    check_vector(next.velocity, {0.0, 1.0, -2.0});
    check_vector(initial.position, {1.0, -2.0, 3.0});
    check_vector(initial.velocity, {4.0, -5.0, 6.0});

    const auto unchanged = advance(initial, acceleration, Seconds{0.0});
    check_vector(unchanged.position, initial.position);
    check_vector(unchanged.velocity, initial.velocity);
    const auto linear = advance(initial, {}, Seconds{0.5});
    check_vector(linear.position, {3.0, -4.5, 6.0});
    check_vector(linear.velocity, initial.velocity);

    const auto gravity = advance(
        State{{0.0, 0.0, 1.0}, {2.0, 0.0, 3.0}},
        {0.0, 0.0, -9.81}, std::chrono::milliseconds{10});
    check_vector(gravity.position, {0.02, 0.0, 1.0295095});
    check_vector(gravity.velocity, {2.0, 0.0, 2.9019});

    const auto split = advance(advance(initial, acceleration, Seconds{0.75}),
                               acceleration, Seconds{1.25});
    check_vector(split.position, next.position);
    check_vector(split.velocity, next.velocity);

    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double largest = std::numeric_limits<double>::max();
    check_throws<std::invalid_argument>([&] {
        (void)advance(initial, acceleration, Seconds{-1.0});
    });
    for (const double invalid : {infinity, -infinity, nan}) {
        check_throws<std::invalid_argument>([&] {
            (void)advance(initial, acceleration, Seconds{invalid});
        });
        for (int axis = 0; axis < 3; ++axis) {
            const Vec3 bad{axis == 0 ? invalid : 0.0,
                           axis == 1 ? invalid : 0.0,
                           axis == 2 ? invalid : 0.0};
            check_throws<std::invalid_argument>([&] {
                (void)advance(State{bad, {}}, {}, Seconds{0.0});
            });
            check_throws<std::invalid_argument>([&] {
                (void)advance(State{{}, bad}, {}, Seconds{1.0});
            });
            check_throws<std::invalid_argument>([&] {
                (void)advance(initial, bad, Seconds{1.0});
            });
        }
    }
    check_throws<std::overflow_error>([&] {
        (void)advance(State{{}, {largest, 0.0, 0.0}}, {}, Seconds{2.0});
    });
    check_throws<std::overflow_error>([&] {
        (void)advance({}, {largest, 0.0, 0.0}, Seconds{2.0});
    });
    const auto stationary = advance({}, {}, Seconds{largest});
    check_vector(stationary.position, {});
    check_vector(stationary.velocity, {});
    std::cout << checks << " checks passed\n";
}
