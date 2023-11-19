#include "brushless.hpp"
#include "moteus/moteus_protocol.h"
#include <units/units.hpp>

namespace mrover {

    BrushlessController::BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controllerName)
        : Controller{nh, std::move(name), std::move(controllerName)} {

        XmlRpc::XmlRpcValue brushlessMotorData;
        assert(mNh.hasParam(std::format("brushless_motors/controllers/{}", mControllerName)));
        mNh.getParam(std::format("brushless_motors/controllers/{}", mControllerName), brushlessMotorData);
        assert(brushlessMotorData.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        mMinVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(
                brushlessMotorData, "min_velocity", 1.0)};
        mMaxVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(
                brushlessMotorData, "max_velocity", 1.0)};

        mMinPosition = Radians{xmlRpcValueToTypeOrDefault<double>(
                brushlessMotorData, "min_position", 1.0)};
        mMaxPosition = Radians{xmlRpcValueToTypeOrDefault<double>(
                brushlessMotorData, "max_position", 1.0)};
    }

    void BrushlessController::setDesiredThrottle(Percent throttle) {
        throttle = std::clamp(throttle, -1_percent, 1_percent);
        setDesiredVelocity(mapThrottleToVelocity(throttle));
    }

    void BrushlessController::setDesiredPosition(Radians position) {
        Revolutions position_revs = std::clamp(position, mMinPosition, mMaxPosition);
        moteus::PositionMode::Command command{
                .position = position_revs.get(),
                .velocity = 0.0,
        };
        moteus::CanFdFrame positionFrame = mController.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    // Position     Velocity
    // Nan          2.0         = spin at 2 rev/s
    // 1.0          0.0         = Stay put at 1 rev round
    // Nan          0.0         = Don't move

    void BrushlessController::setDesiredVelocity(RadiansPerSecond velocity) {
        RevolutionsPerSecond velocity_rev_s = std::clamp(velocity, mMinVelocity, mMaxVelocity);
        // ROS_WARN("%7.3f   %7.3f",
        //  velocity.get(), velocity_rev_s.get());

        moteus::PositionMode::Command command{
                .position = std::numeric_limits<double>::quiet_NaN(),
                .velocity = velocity_rev_s.get(),
        };

        moteus::CanFdFrame positionFrame = mController.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    void BrushlessController::SetStop() {
        moteus::CanFdFrame setStopFrame = mController.MakeStop();
        mDevice.publish_moteus_frame(setStopFrame);
    }

    void BrushlessController::processCANMessage(CAN::ConstPtr const& msg) {
        assert(msg->source == mControllerName);
        assert(msg->destination == mName);
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
            SetStop();
            ROS_WARN("Position timeout hit");
        }
    }

    double BrushlessController::getEffort() {
        // TODO - need to properly set mMeasuredEFfort elsewhere
        return mMeasuredEffort;
    }

    RadiansPerSecond BrushlessController::mapThrottleToVelocity(Percent throttle) {
        std::clamp(throttle, -1_percent, 1_percent);

        // Map the throttle to the velocity range
        return RadiansPerSecond{(throttle.get() + 1.0f) / 2.0f * (mMaxVelocity.get() - mMinVelocity.get()) + mMinVelocity.get()};
    }

} // namespace mrover
