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

        mMinVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_velocity", -1.0)};
        mMaxVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_velocity", 1.0)};

        mMinPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_position", -1.0)};
        mMaxPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_position", 1.0)};
    }

    void BrushlessController::setDesiredThrottle(Percent throttle) {
        updateLastConnection();
        setDesiredVelocity(mapThrottleToVelocity(throttle));
    }

    void BrushlessController::setDesiredPosition(Radians position) {
        updateLastConnection();
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
        updateLastConnection();


        RevolutionsPerSecond velocity_rev_s = std::clamp(velocity, mMinVelocity, mMaxVelocity);
        // ROS_WARN("%7.3f   %7.3f",
        //  velocity.get(), velocity_rev_s.get());
        if (abs(velocity_rev_s).get() < 1e-5) {
            setBrake();
        } else {
            moteus::PositionMode::Command command{
                    .position = std::numeric_limits<double>::quiet_NaN(),
                    .velocity = velocity_rev_s.get(),
            };

            moteus::CanFdFrame positionFrame = mController.MakePosition(command);
            mDevice.publish_moteus_frame(positionFrame);

            if (mControllerName == "joint_de_1")
                ROS_INFO("Brushless Velocity Set Sending to Moteus: %f", command.velocity);
        }
    }

    void BrushlessController::setStop() {
        moteus::CanFdFrame setStopFrame = mController.MakeStop();
        mDevice.publish_moteus_frame(setStopFrame);
    }
    void BrushlessController::setBrake() {
        moteus::CanFdFrame setBrakeFrame = mController.MakeBrake();
        mDevice.publish_moteus_frame(setBrakeFrame);
    }

    void BrushlessController::adjust() {
        moteus::OutputExact::Command outputExactCmd{0.0};
        moteus::CanFdFrame setPositionFrame = mController.MakeOutputExact(outputExactCmd);
        mDevice.publish_moteus_frame(setPositionFrame);
    }

    void BrushlessController::processCANMessage(CAN::ConstPtr const& msg) {
        assert(msg->source == mControllerName);
        assert(msg->destination == mName);
        auto result = moteus::Query::Parse(msg->data.data(), msg->data.size());
        ROS_INFO("controller: %s    %3d p/a/v/t=(%7.3f,%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)",
                 mControllerName.c_str(),
                 result.mode,
                 result.position,
                 result.abs_position,
                 result.velocity,
                 result.torque,
                 result.voltage,
                 result.temperature,
                 result.fault);

        mCurrentPosition = mrover::Radians{
                mrover::Revolutions{result.position}}; // moteus stores position in revolutions.
        mCurrentVelocity = mrover::RadiansPerSecond{
                mrover::RevolutionsPerSecond{result.velocity}}; // moteus stores position in revolutions.

        mErrorState = moteusErrorCodeToErrorState(result.mode, static_cast<ErrorCode>(result.fault));
        mState = moteusModeToState(result.mode);

        if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
            setStop();
            ROS_WARN("Position timeout hit");
        }
    }

    double BrushlessController::getEffort() {
        // TODO - need to properly set mMeasuredEFfort elsewhere.
        // (Art Boyarov): return quiet_Nan, same as Brushed Controller
        return std::numeric_limits<double>::quiet_NaN();
    }

    RadiansPerSecond BrushlessController::mapThrottleToVelocity(Percent throttle) {
        std::clamp(throttle, -1_percent, 1_percent);

        // Map the throttle to the velocity range
        return RadiansPerSecond{(throttle.get() + 1.0f) / 2.0f * (mMaxVelocity.get() - mMinVelocity.get()) + mMinVelocity.get()};
    }

    std::string BrushlessController::moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code) {
        if (motor_mode != moteus::Mode::kFault) return "No Error";
        switch (motor_error_code) {
            case ErrorCode::DmaStreamTransferError:
                return "DMA Stream Transfer Error";
            case ErrorCode::DmaStreamFifiError:
                return "DMA Stream FIFO Error";
            case ErrorCode::UartOverrunError:
                return "UART Overrun Error";
            case ErrorCode::UartFramingError:
                return "UART Framing Error";
            case ErrorCode::UartNoiseError:
                return "UART Noise Error";
            case ErrorCode::UartBufferOverrunError:
                return "UART Buffer Overrun Error";
            case ErrorCode::UartParityError:
                return "UART Parity Error";
            case ErrorCode::CalibrationFault:
                return "Calibration Fault";
            case ErrorCode::MotorDriverFault:
                return "Motor Driver Fault";
            case ErrorCode::OverVoltage:
                return "Over Voltage";
            case ErrorCode::EncoderFault:
                return "Encoder Fault";
            case ErrorCode::MotorNotConfigured:
                return "Motor Not Configured";
            case ErrorCode::PwmCycleOverrun:
                return "PWM Cycle Overrun";
            case ErrorCode::OverTemperature:
                return "Over Temperature";
            case ErrorCode::StartOutsideLimit:
                return "Start Outside Limit";
            case ErrorCode::UnderVoltage:
                return "Under Voltage";
            case ErrorCode::ConfigChanged:
                return "Configuration Changed";
            case ErrorCode::ThetaInvalid:
                return "Theta Invalid";
            case ErrorCode::PositionInvalid:
                return "Position Invalid";
            default:
                return "Unknown Error";
        }
    }

    std::string BrushlessController::moteusModeToState(moteus::Mode motor_mode) {
        switch (motor_mode) {
            case moteus::Mode::kStopped:
                return "Motor Stopped";
            case moteus::Mode::kFault:
                return "Motor Fault";
            case moteus::Mode::kPwm:
                return "PWM Operating Mode";
            case moteus::Mode::kVoltage:
                return "Voltage Operating Mode";
            case moteus::Mode::kVoltageFoc:
                return "Voltage FOC Operating Mode";
            case moteus::Mode::kVoltageDq:
                return "Voltage DQ Operating Mode";
            case moteus::Mode::kCurrent:
                return "Current Operating Mode";
            case moteus::Mode::kPosition:
                return "Position Operating Mode";
            case moteus::Mode::kPositionTimeout:
                return "Position Timeout";
            case moteus::Mode::kZeroVelocity:
                return "Zero Velocity";
            default:
                return "Unknown Mode";
        }
    }
} // namespace mrover
