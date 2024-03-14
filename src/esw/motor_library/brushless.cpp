#include "brushless.hpp"
#include "moteus/moteus_protocol.h"
#include <moteus/moteus_multiplex.h>
#include <units/units.hpp>

namespace mrover {

    BrushlessController::BrushlessController(ros::NodeHandle const& nh, std::string name, std::string controllerName)
        : Controller{nh, std::move(name), std::move(controllerName)} {

        XmlRpc::XmlRpcValue brushlessMotorData;
        assert(mNh.hasParam(std::format("brushless_motors/controllers/{}", mControllerName)));
        mNh.getParam(std::format("brushless_motors/controllers/{}", mControllerName), brushlessMotorData);
        assert(brushlessMotorData.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        mVelocityMultiplier = Dimensionless{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "velocity_multiplier", 1.0)};
        if (mVelocityMultiplier.get() == 0) {
            throw std::runtime_error("Velocity multiplier can't be 0!");
        }

        mMinVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_velocity", -1.0)};
        mMaxVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_velocity", 1.0)};

        mMinPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_position", -1.0)};
        mMaxPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_position", 1.0)};

        mMaxTorque = xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_torque", 0.3);
        mWatchdogTimeout = xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "watchdog_timeout", 0.1);

        limitSwitch0Present = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_present", false);
        limitSwitch1Present = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_present", false);
        limitSwitch0Enabled = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_enabled", true);
        limitSwitch1Enabled = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_enabled", true);
        limitSwitch0LimitsFwd = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_limits_fwd", false);
        limitSwitch1LimitsFwd = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_limits_fwd", false);
        limitSwitch0ActiveHigh = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_is_active_high", true);
        limitSwitch1ActiveHigh = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_is_active_high", true);
        limitSwitch0UsedForReadjustment = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_used_for_readjustment", false);
        limitSwitch1UsedForReadjustment = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_used_for_readjustment", false);
        limitSwitch0ReadjustPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "limit_0_readjust_position", 0.0)};
        limitSwitch1ReadjustPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "limit_1_readjust_position", 0.0)};

    }

    auto BrushlessController::setDesiredThrottle(Percent throttle) -> void {
        setDesiredVelocity(mapThrottleToVelocity(throttle));
    }

    auto BrushlessController::setDesiredPosition(Radians position) -> void {
        // only check for limit switches if at least one limit switch exists and is enabled
        if ((limitSwitch0Enabled && limitSwitch0Present) || (limitSwitch1Enabled && limitSwitch0Present)) {
            sendQuery();

            MoteusLimitSwitchInfo limitSwitchInfo = getPressedLimitSwitchInfo();
            if ((mCurrentPosition < position && limitSwitchInfo.isFwdPressed) || (mCurrentPosition > position && limitSwitchInfo.isBwdPressed)) {
                setBrake();
                return;
            }
        }

        Revolutions position_revs = std::clamp(position, mMinPosition, mMaxPosition);
        moteus::PositionMode::Command command{
                .position = position_revs.get(),
                .velocity = 0.0,
                .maximum_torque = mMaxTorque,
                .watchdog_timeout = mWatchdogTimeout,
        };
        moteus::CanFdFrame positionFrame = mController.MakePosition(command);
        mDevice.publish_moteus_frame(positionFrame);
    }

    // Position     Velocity
    // Nan          2.0         = spin at 2 rev/s
    // 1.0          0.0         = Stay put at 1 rev round
    // Nan          0.0         = Don't move

    auto BrushlessController::setDesiredVelocity(RadiansPerSecond velocity) -> void {
        // only check for limit switches if at least one limit switch exists and is enabled
        if ((limitSwitch0Enabled && limitSwitch0Present) || (limitSwitch1Enabled && limitSwitch0Present)) {
            sendQuery();

            MoteusLimitSwitchInfo limitSwitchInfo = getPressedLimitSwitchInfo();
            if ((velocity > Radians{0} && limitSwitchInfo.isFwdPressed) || (velocity < Radians{0} && limitSwitchInfo.isBwdPressed)) {
                setBrake();
                return;
            }
        }
        

        velocity = velocity * mVelocityMultiplier;

        // ROS_INFO("my velocity rev s = %f", velocity.get()); 

        RevolutionsPerSecond velocity_rev_s = std::clamp(velocity, mMinVelocity, mMaxVelocity);
        if (abs(velocity_rev_s).get() < 1e-5) {
            setBrake();
            ROS_INFO("In brake mode because velocity_rev_s = %f", velocity_rev_s.get());
        } else {
            moteus::PositionMode::Command command{
                    .position = std::numeric_limits<double>::quiet_NaN(),
                    .velocity = velocity_rev_s.get(),
                    .maximum_torque = mMaxTorque,
                    .watchdog_timeout = mWatchdogTimeout,
            };

            moteus::CanFdFrame positionFrame = mController.MakePosition(command);
            ROS_INFO("sending this velocity: %f", command.velocity);
            mDevice.publish_moteus_frame(positionFrame);
        }
    }

    auto BrushlessController::setStop() -> void {
        moteus::CanFdFrame setStopFrame = mController.MakeStop();
        mDevice.publish_moteus_frame(setStopFrame);
    }

    auto BrushlessController::setBrake() -> void {
        moteus::CanFdFrame setBrakeFrame = mController.MakeBrake();
        mDevice.publish_moteus_frame(setBrakeFrame);
    }

    auto BrushlessController::getPressedLimitSwitchInfo() -> MoteusLimitSwitchInfo {
        /*
        Testing 2/9:
        - Connected limit switch (common is input(black), NC to ground)
        - Configured moteus to have all pins set as digital_input and pull_up
        - When limit switch not pressed, aux2 = 0xD
        - When limit switch pressed, aux1 = 0xF
        - Note: has to be active high, so in this scenario we have to flip this bit round.
        - This was connected to just one moteus board, not the one with a motor on it.

        Stuff for Limit Switches (from Guthrie)
        - Read from config about limit switch settings
        - Either 1 or 0 not forward/backward

        - Add a member variable in Brushless.hpp to store limit switch value. 
          pdate this limit switch variable every round in ProcessCANMessage.
        
        - Note: we can get aux pin info even without sending a query command. 
          Tested with sending velocity commands.
        */
        // TODO - implement this
        MoteusLimitSwitchInfo result{};
    
        result.isFwdPressed = false;
        result.isBwdPressed = false;

        // TODO do both switch0 and switch1 use aux2?
        if (limitSwitch0Present && limitSwitch0Enabled) {
            int bitMask = 2; // 0b0010
            bool gpioState = bitMask & moteusAux1Info;
            mLimitHit.at(0) = gpioState == limitSwitch0ActiveHigh;
        }
        if (limitSwitch1Present && limitSwitch1Enabled) {
            int bitMask = 2; // 0b0010
            bool gpioState = bitMask & moteusAux2Info;
            mLimitHit.at(1) = gpioState == limitSwitch1ActiveHigh;
        }

        result.isFwdPressed = (mLimitHit.at(0) && limitSwitch0LimitsFwd) || (mLimitHit.at(1) && limitSwitch1LimitsFwd);
        result.isBwdPressed = (mLimitHit.at(0) && !limitSwitch0LimitsFwd) || (mLimitHit.at(1) && !limitSwitch1LimitsFwd);

        if (result.isFwdPressed) {
            adjust(limitSwitch0ReadjustPosition);
        }
        else if (result.isBwdPressed) {
            adjust(limitSwitch1ReadjustPosition);
        }

        return result;
    }

    auto BrushlessController::adjust(Radians commandedPosition) -> void {
        Revolutions commandedPosition_rev = std::clamp(commandedPosition, mMinPosition, mMaxPosition);
        moteus::OutputExact::Command command{
                .position = commandedPosition_rev.get(),
        };
        moteus::OutputExact::Command outputExactCmd{command};
        moteus::CanFdFrame setPositionFrame = mController.MakeOutputExact(outputExactCmd);
        mDevice.publish_moteus_frame(setPositionFrame);
    }

    auto BrushlessController::sendQuery() -> void {
        moteus::Query::Format qFormat{};
        qFormat.aux1_gpio = moteus::kInt8;
        qFormat.aux2_gpio = moteus::kInt8;
        moteus::CanFdFrame queryFrame = mController.MakeQuery(&qFormat);
        mDevice.publish_moteus_frame(queryFrame);
    }

    auto BrushlessController::processCANMessage(CAN::ConstPtr const& msg) -> void {
        assert(msg->source == mControllerName);
        assert(msg->destination == mName);
        auto result = moteus::Query::Parse(msg->data.data(), msg->data.size());
        ROS_INFO("controller: %s    %3d p/a/v/t=(%7.3f,%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d) GPIO: Aux1-%X , Aux2-%X",
                 mControllerName.c_str(),
                 result.mode,
                 result.position,
                 result.abs_position,
                 result.velocity,
                 result.torque,
                 result.voltage,
                 result.temperature,
                 result.fault,  
                 result.aux1_gpio,
                 result.aux2_gpio
                 );

        if (mControllerName == "joint_de_0" || mControllerName == "joint_de_1") {
            mCurrentPosition = mrover::Revolutions{result.aux1_gpio};   //get value of absolute encoder if its joint_de0/1
        }

        else {
            mCurrentPosition = mrover::Revolutions{result.position}; // moteus stores position in revolutions.
        }
        
        mCurrentVelocity = mrover::RevolutionsPerSecond{result.velocity} / mVelocityMultiplier; // moteus stores position in revolutions.

        mErrorState = moteusErrorCodeToErrorState(result.mode, static_cast<ErrorCode>(result.fault));
        mState = moteusModeToState(result.mode);

        moteusAux1Info = (result.aux1_gpio) ? result.aux1_gpio : moteusAux1Info;
        moteusAux2Info = (result.aux1_gpio) ? result.aux2_gpio : moteusAux2Info;

        if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
            setStop();
            ROS_WARN("Position timeout hit");
        }
    }

    auto BrushlessController::getEffort() -> double {
        // TODO - need to properly set mMeasuredEFfort elsewhere.
        // (Art Boyarov): return quiet_Nan, same as Brushed Controller
        return std::numeric_limits<double>::quiet_NaN();
    }

    auto BrushlessController::mapThrottleToVelocity(Percent throttle) const -> RadiansPerSecond {
        std::clamp(throttle, -1_percent, 1_percent);

        // Map the throttle to the velocity range
        return RadiansPerSecond{(throttle.get() + 1.0f) / 2.0f * (mMaxVelocity.get() - mMinVelocity.get()) + mMinVelocity.get()};
    }

    auto BrushlessController::moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code) -> std::string {
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

    auto BrushlessController::moteusModeToState(moteus::Mode motor_mode) -> std::string {
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
