#include "brushless.hpp"

void BrushlessController::update(const std::vector<uint8_t> &frame) {
    if (frame.empty()) {
        return; // TOOD
    }
    ROS_INFO("TODO - need to update based on frame.");
}

void BrushlessController::set_desired_throttle(double throttle) {
    throttle = std::clamp(throttle, -1.0, 1.0);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to convert from %f to rev/s and send torque %f.", throttle, torque);
    can_manager.send_raw_data(can_frame);
}

void BrushlessController::set_desired_position(double position) {
    position = std::clamp(position, min_position, max_position);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to send %f.", position);
    // For moteus, it needs to send rev/s.
    can_manager.send_raw_data(can_frame);
}

void BrushlessController::set_desired_velocity(double velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to send %f.", velocity);
    // For moteus, it needs to send rev/s.
    can_manager.send_raw_data(can_frame);
}

MotorType BrushlessController::get_type() {
    return {};
}
