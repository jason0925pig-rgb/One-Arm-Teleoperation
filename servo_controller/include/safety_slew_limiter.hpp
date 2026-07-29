#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace one_arm_safety {

struct SlewStep {
    double position;
    double velocity;
};

inline SlewStep acceleration_limited_step(
    double position,
    double velocity,
    double target,
    double maximum_velocity,
    double maximum_acceleration,
    double dt) {
    if (
        !std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(target) || !std::isfinite(maximum_velocity) ||
        !std::isfinite(maximum_acceleration) || !std::isfinite(dt) ||
        maximum_velocity <= 0.0 || maximum_acceleration <= 0.0 || dt <= 0.0) {
        throw std::invalid_argument("invalid acceleration-limiter input");
    }

    const double error = target - position;
    const double stopping_speed =
        std::sqrt(2.0 * maximum_acceleration * std::abs(error));
    const double desired_speed = std::min(maximum_velocity, stopping_speed);
    const double desired_velocity =
        error > 0.0 ? desired_speed : (error < 0.0 ? -desired_speed : 0.0);
    const double maximum_velocity_change = maximum_acceleration * dt;
    const double next_velocity = std::clamp(
        velocity + std::clamp(
            desired_velocity - velocity,
            -maximum_velocity_change,
            maximum_velocity_change),
        -maximum_velocity,
        maximum_velocity);

    return SlewStep{
        position + next_velocity * dt,
        next_velocity,
    };
}

}  // namespace one_arm_safety
