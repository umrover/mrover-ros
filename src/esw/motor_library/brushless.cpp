#include "brushless.hpp"
#include "moteus/moteus_protocol.h"
#include <units/units.hpp>

namespace mrover {

    BrushlessController::BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controllerName)
        : Controller{nh, std::move(name), std::move(controllerName)} {

        mMinVelocity = RadiansPerSecond{-1.0f};
        mMaxVelocity = RadiansPerSecond{1.0f};

        // TODO: change
    }

    void BrushlessController::setDesiredThrottle(Percent throttle) {
        throttle = std::clamp(throttle, -1_percent, 1_percent);
        setDesiredVelocity(mMaxVelocity * throttle);
    }

    void BrushlessController::setDesiredPosition(Radians position) {
        Revolutions position_revs = std::clamp(position, mMinPosition, mMaxPosition);
        moteus::Controller::Options options;
        moteus::Controller controller{options};
        controller.SetStop();
        moteus::PositionMode::Command command{
                .position = position_revs.get(),
                .velocity = 0.0,
        };
        moteus::CanFdFrame positionFrame = controller.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    // Position     Velocity
    // Nan          2.0         = spin at 2 rev/s
    // 1.0          0.0         = Stay put at 1 rev round
    // Nan          0.0         = Don't move

    void BrushlessController::setDesiredVelocity(RadiansPerSecond velocity) {
        RevolutionsPerSecond velocity_rev_s = std::clamp(velocity, mMinVelocity, mMaxVelocity);
        ROS_WARN("%7.3f   %7.3f",
                 velocity.get(), velocity_rev_s.get());

        moteus::PositionMode::Command command{
                .position = std::numeric_limits<double>::quiet_NaN(),
                .velocity = velocity_rev_s.get(),
        };

        moteus::CanFdFrame positionFrame = mController.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    void BrushlessController::SetStop() {

        moteus::Controller::Options options;
        moteus::Controller controller{options};
        moteus::CanFdFrame setStopFrame = controller.MakeStop();
        mDevice.publish_moteus_frame(setStopFrame);
    }

    void BrushlessController::processCANMessage(CAN::ConstPtr const& msg) {
        auto result = moteus::Query::Parse(msg->data.data(), msg->data.size());
        ROS_INFO("%3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)",
                 result.mode,
                 result.position,
                 result.velocity,
                 result.torque,
                 result.voltage,
                 result.temperature,
                 result.fault);

        if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
            moteus::CanFdFrame stopFrame = mController.MakeStop();
            mDevice.publish_moteus_frame(stopFrame);
            ROS_WARN("Position timeout hit");
        }
    }

    double BrushlessController::getEffort() {
        // TODO - need to properly set mMeasuredEFfort elsewhere
        return mMeasuredEffort;
    }

} // namespace mrover
