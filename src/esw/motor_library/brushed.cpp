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

    void BrushedController::set_desired_throttle(Percent throttle) {
        assert(throttle >= -1_percent && throttle <= 1_percent);

        can_manager.send_data("test", ThrottleCommand{.throttle = throttle});
    }

    void BrushedController::set_desired_position(Radians position) {
        assert(position >= min_position && position <= max_position);

        can_manager.send_data("test", PositionCommand{.position = position});
    }

    void BrushedController::set_desired_velocity(RadiansPerSecond velocity) {
        assert(velocity >= min_velocity && velocity <= max_velocity);

        can_manager.send_data("test", VelocityCommand{.velocity = velocity});
    }

} // namespace mrover
