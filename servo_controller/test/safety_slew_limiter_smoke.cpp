#include "safety_slew_limiter.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

int main() {
    constexpr double dt = 0.008;
    constexpr double maximum_velocity = 0.10;
    constexpr double maximum_acceleration = 0.20;

    double position = 0.0;
    double velocity = 0.0;
    for (int index = 0; index < 2000; ++index) {
        const double previous_velocity = velocity;
        const auto step = one_arm_safety::acceleration_limited_step(
            position,
            velocity,
            1.0,
            maximum_velocity,
            maximum_acceleration,
            dt);
        position = step.position;
        velocity = step.velocity;
        assert(std::abs(velocity) <= maximum_velocity + 1e-12);
        assert(
            std::abs(velocity - previous_velocity) <=
            maximum_acceleration * dt + 1e-12);
    }
    assert(std::abs(position - 1.0) < 0.001);

    const double velocity_before_reverse = velocity;
    const auto reverse = one_arm_safety::acceleration_limited_step(
        position,
        velocity,
        -1.0,
        maximum_velocity,
        maximum_acceleration,
        dt);
    assert(
        std::abs(reverse.velocity - velocity_before_reverse) <=
        maximum_acceleration * dt + 1e-12);

    bool rejected = false;
    try {
        (void)one_arm_safety::acceleration_limited_step(
            0.0, 0.0, 1.0, maximum_velocity, maximum_acceleration, 0.0);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    assert(rejected);
    return 0;
}
