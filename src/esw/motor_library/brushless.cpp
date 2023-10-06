#include "brushless.hpp"

void BrushlessController::update(const std::vector<uint8_t> &frame) {
    if (frame.empty()) {
        return; // TODO
    }
    else {
        // TODO - TEMPORARY
        velocity = 0;
    }
    ROS_INFO("TODO - need to update based on frame.");
}

void BrushlessController::set_desired_throttle(float throttle) {
    throttle = std::clamp(throttle, -1.0f, 1.0f);
    std::vector<uint8_t> can_frame = createFloatMessage(throttle);
    // TODO - need to convert from throttle to rev/s 
    can_manager.send_raw_data("throttle_cmd", can_frame);
}

void BrushlessController::set_desired_position(float position) {
    position = std::clamp(position, min_position, max_position);
    // TODO - need to convert to use revs 
    std::vector<uint8_t> can_frame = createFloatMessage(position);
    can_manager.send_raw_data("position_cmd", can_frame);
}

void BrushlessController::set_desired_velocity(float velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    // TODO - need to convert to use rev/s
    std::vector<uint8_t> can_frame = createFloatMessage(velocity);
    can_manager.send_raw_data("velocity_cmd", can_frame);
}
