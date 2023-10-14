#include "brushed.hpp"

void BrushedController::update(std::span<std::byte const> frame) {
    if (frame.empty()) {
        return;
    } else {
        // TODO - TEMPORARY
        velocity = 0;
    }
    ROS_INFO("TODO - need to update based on frame.");
}

void BrushedController::set_desired_throttle(float throttle) {
    throttle = std::clamp(throttle, -1.0f, 1.0f);
    can_manager.send_data("throttle_cmd", throttle);
}

void BrushedController::set_desired_position(float position) {
    position = std::clamp(position, min_position, max_position);
    can_manager.send_data("position_cmd", position);
}

void BrushedController::set_desired_velocity(float velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    can_manager.send_data("velocity_cmd", velocity);
}