#include "brushed.hpp"

void BrushedController::update(uint64_t frame) {
    if (frame == 0) {
        return;
        // TODO
    }
}

void BrushedController::set_desired_throttle(double throttle) {
    uint64_t can_frame = 0;
    if (throttle == 0) {
        can_frame = 1;  // todo
    }
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_position(double position) {
    uint64_t can_frame = 0;
    if (position == 0) {
        can_frame = 1;  // todo
    }
    can_manager.send_raw_data(can_frame);
}

void BrushedController::set_desired_velocity(double velocity) {
    uint64_t can_frame = 0;
    if (velocity == 0) {
        can_frame = 1;  // todo
    }
    can_manager.send_raw_data(can_frame);
}

MotorType BrushedController::get_type() {
    return {};
}
