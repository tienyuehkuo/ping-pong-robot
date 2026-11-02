#include <ping_pong/motion.hpp>

#include <chrono>
#include <exception>
#include <iostream>

int main() {
    const ping_pong::State initial{{0.0, 0.0, 1.0}, {2.0, 0.0, 3.0}};
    const ping_pong::Vec3 gravity{0.0, 0.0, -9.81}; // z points up
    try {
        const auto next = ping_pong::advance(
            initial, gravity, std::chrono::milliseconds{10});
        std::cout << "Position (m): " << next.position.x << ", "
                  << next.position.y << ", " << next.position.z << '\n'
                  << "Velocity (m/s): " << next.velocity.x << ", "
                  << next.velocity.y << ", " << next.velocity.z << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
