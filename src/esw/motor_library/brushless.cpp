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

    void BrushlessController::set_desired_throttle(Percent throttle) {
        throttle = std::clamp(throttle, -1_percent, 1_percent);
        // TODO - need to convert from throttle to rev/s
        // TODO - create a CAN frame
    }

    void BrushlessController::set_desired_position(Radians position) {
        position = std::clamp(position, min_position, max_position);
        // TODO - need to convert to use revs
    }
    void BrushlessController::set_desired_velocity(RadiansPerSecond velocity) {
        velocity = std::clamp(velocity, min_velocity, max_velocity);
        moteus::Controller::Options options;
        options.id = 1;

        moteus::Controller controller(options);
        auto transport = moteus::Controller::MakeSingletonTransport({});

        // Command a stop to the controller in order to clear any faults.
        controller.SetStop();

        moteus::PositionMode::Command cmd;

        // Here we will just command a position of NaN and a velocity of
        // 0.0.  This means "hold position wherever you are".

        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = velocity.get();

        auto CANfd = controller.MakePosition(cmd);

        // TODO - need to convert to use rev/s
        can_manager.send_moteus_data(CANfd);
    }
} // namespace mrover
