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
    }

    void BrushlessController::set_desired_velocity(RadiansPerSecond velocity) {
        velocity = std::clamp(velocity, mMinVelocity, mMaxVelocity);

        moteus::Controller::Options options;
        moteus::Controller controller{options};

        // Here we will just command a position of NaN and a velocity of
        // 0.0.  This means "hold position wherever you are".

        moteus::PositionMode::Command command{
                .position = std::numeric_limits<double>::quiet_NaN(),
                .velocity = velocity.get(),
        };
        moteus::CanFdFrame positionFrame = controller.MakePosition(command);

        // TODO - need to convert to use rev/s
        mDevice.publish_moteus_frame(positionFrame);
    }

    void BrushlessController::processCANMessage(CAN::ConstPtr const& msg) {
    }

    double BrushlessController::getEffort() {
        // TODO - actually do something
        return mMeasuredEffort;
    }

} // namespace mrover
