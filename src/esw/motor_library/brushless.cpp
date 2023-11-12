#include "brushless.hpp"

namespace mrover {

    BrushlessController::BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controller_name)
        : Controller{nh, std::move(name), std::move(controller_name)} {

        // TODO: change
        torque = 0.3f;
    }

    void BrushlessController::set_desired_throttle(Percent throttle) {
        throttle = std::clamp(throttle, -1_percent, 1_percent);
        // TODO - need to convert from throttle to rev/s
        // TODO - create a CAN frame
    }

    void BrushlessController::set_desired_position(Radians position) {
        position = std::clamp(position, mMinPosition, mMaxPosition);
        // TODO - need to convert to use revs

        moteus::Controller::Options options;
        moteus::Controller controller{options};

        moteus::PositionMode::Command command{
                .position = position.get(),
                .velocity = 0.0,
        };
        moteus::CanFdFrame positionFrame = controller.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    // Position     Velocity
    // Nan          2.0         = spin at 2 rev/s
    // 1.0          0.0         = Stay put at 1 rev round
    // Nan          0.0         = Don't move

    void BrushlessController::set_desired_velocity(RadiansPerSecond velocity) {
        // TODO: Convert radians per second to revolutions per second
        velocity = std::clamp(velocity, mMinVelocity, mMaxVelocity);

        moteus::Controller::Options options;
        moteus::Controller controller{options};

        moteus::PositionMode::Command command{
                .position = std::numeric_limits<double>::quiet_NaN(),
                .velocity = velocity.get(),
        };
        moteus::CanFdFrame positionFrame = controller.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    void BrushlessController::processCANMessage(CAN::ConstPtr const& msg) {
    }

} // namespace mrover
