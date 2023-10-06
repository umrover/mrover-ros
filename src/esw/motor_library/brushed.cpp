#include "brushed.hpp"

void BrushedController::update(const std::vector<uint8_t> &frame) {
    if (frame.empty()) {
        return;
    }
    else {
        // TODO - TEMPORARY
        velocity = 0;
    }
    ROS_INFO("TODO - need to update based on frame.");
}

void BrushedController::set_desired_throttle(float throttle) {
    throttle = std::clamp(throttle, -1.0f, 1.0f);
    std::vector<uint8_t> can_frame = createFloatMessage(throttle);

    can_manager.send_raw_data("throttle_cmd", can_frame);
}

void BrushedController::set_desired_position(float position) {
    position = std::clamp(position, min_position, max_position);
    std::vector<uint8_t> can_frame = createFloatMessage(position);

    can_manager.send_raw_data("position_cmd", can_frame);
}

void BrushedController::set_desired_velocity(float velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    std::vector<uint8_t> can_frame = createFloatMessage(velocity);

    can_manager.send_raw_data("velocity_cmd", can_frame);
}