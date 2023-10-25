#include "brushless.hpp"

namespace mrover {

    void BrushlessController::update(std::span<std::byte const> frame) {
        if (frame.empty()) {
            return; // TODO
        } else {
            // TODO - TEMPORARY
            velocity = 0_rad_per_s;
        }
        ROS_INFO("TODO - need to update based on frame.");
    }

    void BrushlessController::set_desired_throttle(Dimensionless throttle) {
        throttle = std::clamp(throttle, Dimensionless{-1}, Dimensionless{1});
        // TODO - need to convert from throttle to rev/s
        can_manager.send_data("throttle_cmd", throttle);
    }

    void BrushlessController::set_desired_position(Radians position) {
        position = std::clamp(position, min_position, max_position);
        // TODO - need to convert to use revs
    }
    void BrushlessController::set_desired_velocity(float velocity) {
        velocity = std::clamp(velocity, min_velocity, max_velocity);
        // TODO - need to convert to use rev/s
        can_manager.send_data("velocity_cmd", velocity);
    }
} // namespace mrover
int main() {
    mrover::BrushlessController bc();
    uint8_t[64] velocity = bc.test_set_velocity(5.0);
    std::cout << velocity;


    //can_manager.send_data("velocity_cmd", velocity);
}
