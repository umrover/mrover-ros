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

    void BrushedController::send_brushed_message(InBoundMessage const& message) {
        // TODO(quintin): we may need to do some bit hacking here
        can_manager.send_data(name, message);
    }

    void BrushedController::set_desired_throttle(Percent throttle) {
        assert(throttle >= -1_percent && throttle <= 1_percent);

        send_brushed_message("throttle_cmd", ThrottleCommand{.throttle = throttle});
    }

    void BrushedController::set_desired_position(Radians position) {
        assert(position >= min_position && position <= max_position);

        send_brushed_message("position_cmd", PositionCommand{.position = position});
    }

    void BrushedController::set_desired_velocity(RadiansPerSecond velocity) {
        assert(velocity >= min_velocity && velocity <= max_velocity);

        send_brushed_message("velocity_cmd", VelocityCommand{.velocity = velocity});
    }

} // namespace mrover
