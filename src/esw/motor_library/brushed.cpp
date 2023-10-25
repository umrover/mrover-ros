#include "brushed.hpp"

namespace mrover {

    void BrushedController::update(std::span<std::byte const> frame) {
        if (frame.empty()) {
            return;
        } else {
            // TODO - TEMPORARY
            velocity = 0_rad_per_s;
        }
        ROS_INFO("TODO - need to update based on frame.");
    }

    void BrushedController::set_desired_throttle(Dimensionless throttle) {
        throttle = std::clamp(throttle, Dimensionless{-1}, Dimensionless{1});
        can_manager.send_data("throttle_cmd", throttle);
    }

    void BrushedController::set_desired_position(Radians position) {
        position = std::clamp(position, min_position, max_position);
        can_manager.send_data("position_cmd", position);
    }

    void BrushedController::set_desired_velocity(RadiansPerSecond velocity) {
        velocity = std::clamp(velocity, min_velocity, max_velocity);
        can_manager.send_data("velocity_cmd", velocity);
    }

} // namespace mrover
