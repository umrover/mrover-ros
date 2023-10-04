#include "brushed.hpp"

void BrushedController::update(uint64_t frame) {
    ROS_INFO("TODO - need to update based on %lu.", frame);
}

void BrushedController::set_desired_throttle(double throttle) {
    throttle = std::clamp(throttle, -1.0, 1.0);
    uint64_t can_frame = 0;
    ROS_INFO("TODO - need to send %f.", throttle);
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_position(double position) {
    position = std::clamp(position, min_position, max_position);
    uint64_t can_frame = 0;
    ROS_INFO("TODO - need to send %f.", position);
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_velocity(double velocity) {
    velocity = std::clamp(velocity, min_velocity, max_velocity);
    uint64_t can_frame = 0;
    ROS_INFO("TODO - need to send %f.", velocity);
    can_manager.send_raw_data(can_frame);
}

MotorType BrushedController::get_type() {
    return {};
}
