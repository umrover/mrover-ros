#include "brushed.hpp"

void BrushedController::update(const std::vector<uint8_t> &frame) {
    if (frame.empty()) {
        return;
    }
    ROS_INFO("TODO - need to update based on frame.");
}

void BrushedController::set_desired_throttle(double throttle) {
    throttle = std::clamp(throttle, -1.0, 1.0);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to send %f.", throttle);
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_position(double position) {
    position = std::clamp(position, min_position, max_position);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to send %f.", position);
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_velocity(double velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    std::vector<uint8_t> can_frame = {0};
    ROS_INFO("TODO - need to send %f.", velocity);
    can_manager.send_raw_data(can_frame);
}

MotorType BrushedController::get_type() {
    return {};
}
