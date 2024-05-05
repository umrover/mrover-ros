#include "brushed.hpp"

namespace mrover {

    BrushedController::BrushedController(ros::NodeHandle const& nh, std::string masterName, std::string controllerName)
        : ControllerBase{nh, std::move(masterName), std::move(controllerName)} {

        XmlRpc::XmlRpcValue brushedMotorData;
        assert(mNh.hasParam(std::format("brushed_motors/controllers/{}", mControllerName)));
        mNh.getParam(std::format("brushed_motors/controllers/{}", mControllerName), brushedMotorData);
        assert(brushedMotorData.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        mConfigCommand.gear_ratio = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "gear_ratio");

        for (std::size_t i = 0; i < 4; ++i) {
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i, xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_present", i), false));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.enabled, i, xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_enabled", i), false));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.active_high, i, xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_is_active_high", i), true)); // might switch default value to false depending on wiring
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.limits_forward, i, xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_limits_fwd", i), true));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.use_for_readjustment, i, xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_used_for_readjustment", i), true));
            mConfigCommand.limit_switch_info.limit_readj_pos.at(i) = Radians{static_cast<double>(brushedMotorData[std::format("limit_{}_readjust_position", i)])};
        }

        mConfigCommand.is_inverted = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "is_inverted", false);

        mConfigCommand.enc_info.quad_present = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "quad_present", false);
        mConfigCommand.enc_info.quad_ratio = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "quad_ratio", 1.0);

        mConfigCommand.enc_info.abs_present = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "abs_present", false);
        mConfigCommand.enc_info.abs_ratio = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "abs_ratio", 1.0);
        mConfigCommand.enc_info.abs_offset = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "abs_offset", 0.0)};

        auto driver_voltage = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "driver_voltage");
        assert(driver_voltage > 0);
        auto motor_max_voltage = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "motor_max_voltage");
        assert(0 < motor_max_voltage && motor_max_voltage <= driver_voltage);

        mConfigCommand.max_pwm = motor_max_voltage / driver_voltage;

        mConfigCommand.limit_switch_info.limit_max_forward_position = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "limit_max_forward_pos", false);
        mConfigCommand.limit_switch_info.limit_max_backward_position = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "limit_max_backward_pos", false);

        mConfigCommand.min_position = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "min_position", -std::numeric_limits<double>::infinity())};
        mConfigCommand.max_position = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_position", std::numeric_limits<double>::infinity())};

        mConfigCommand.min_velocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "min_velocity", -std::numeric_limits<double>::infinity())};
        mConfigCommand.max_velocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_velocity", std::numeric_limits<double>::infinity())};

        mPositionGains.p = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_p", 0.0);
        mPositionGains.i = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_i", 0.0);
        mPositionGains.d = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_d", 0.0);
        mPositionGains.ff = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_ff", 0.0);

        mVelocityGains.p = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_p", 0.0);
        mVelocityGains.i = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_i", 0.0);
        mVelocityGains.d = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_d", 0.0);
        mVelocityGains.ff = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_ff", 0.0);

        for (int i = 0; i < 4; ++i) {
            mHasLimit |= xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_enabled", i), false);
        }
        mCalibrationThrottle = Percent{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "calibration_throttle", 0.0)};
        mErrorState = "Unknown";
        mState = "Unknown";
    }

    auto BrushedController::setDesiredThrottle(Percent throttle) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        mDevice.publish_message(InBoundMessage{ThrottleCommand{.throttle = throttle}});
    }

    auto BrushedController::setDesiredPosition(Radians position) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mConfigCommand.min_position && position <= mConfigCommand.max_position);

        mDevice.publish_message(InBoundMessage{PositionCommand{
                .position = position,
                .p = static_cast<float>(mPositionGains.p),
                .i = static_cast<float>(mPositionGains.i),
                .d = static_cast<float>(mPositionGains.d),
        }});
    }

    auto BrushedController::setDesiredVelocity(RadiansPerSecond velocity) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(velocity >= mConfigCommand.min_velocity && velocity <= mConfigCommand.max_velocity);

        mDevice.publish_message(InBoundMessage{VelocityCommand{
                .velocity = velocity,
                .p = static_cast<float>(mVelocityGains.p),
                .i = static_cast<float>(mVelocityGains.i),
                .d = static_cast<float>(mVelocityGains.d),
                .ff = static_cast<float>(mVelocityGains.ff),
        }});
    }

    auto BrushedController::sendConfiguration() -> void {
        mDevice.publish_message(InBoundMessage{mConfigCommand});

        // Need to await configuration. Can NOT directly set mIsConfigured to true.
    }

    auto BrushedController::adjust(Radians position) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mConfigCommand.min_position && position <= mConfigCommand.max_position);

        mDevice.publish_message(InBoundMessage{AdjustCommand{.position = position}});
    }

    auto BrushedController::processMessage(ControllerDataState const& state) -> void {
        mCurrentPosition = state.position;
        mCurrentVelocity = state.velocity;
        ConfigCalibErrorInfo configCalibErrInfo = state.config_calib_error_data;
        mIsConfigured = configCalibErrInfo.configured;
        mIsCalibrated = configCalibErrInfo.calibrated;
        mErrorState = errorToString(configCalibErrInfo.error);
        auto const& [_, hit] = state.limit_switches;
        for (std::size_t i = 0; i < mLimitHit.size(); ++i) {
            mLimitHit.at(i) = GET_BIT_AT_INDEX(hit, i);
        }
        if (mIsCalibrated) {
            mState = "Armed";
        } else if (mIsConfigured) {
            mState = "Not Calibrated";
        }
        else {
            mState = "Not Configured";
        }
    }

    auto BrushedController::processCANMessage(CAN::ConstPtr const& msg) -> void {
        assert(msg->source == mControllerName);
        assert(msg->destination == mMasterName);

        OutBoundMessage const& message = *reinterpret_cast<OutBoundMessage const*>(msg->data.data());

        // This calls the correct process function based on the current value of the alternative
        std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
    }

    auto BrushedController::getEffort() -> double {
        return std::numeric_limits<double>::quiet_NaN();
    }

    auto BrushedController::errorToString(BDCMCErrorInfo errorCode) -> std::string {
        switch (errorCode) {
            case BDCMCErrorInfo::NO_ERROR:
                return "NO_ERROR";
            case BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED:
                return "DEFAULT_START_UP_NOT_CONFIGURED";
            case BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED:
                return "RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED";
            case BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED:
                return "RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED";
            case BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS:
                return "OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS";
            case BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS:
                return "RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS";
            default:
                return "UNKNOWN_ERROR_CODE";
        }
    }

    auto BrushedController::calibrateServiceCallback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) -> bool {
        if (!mHasLimit) {
            res.success = false;
            res.message = std::format("{} does not have limit switches, cannot calibrate", mControllerName);
            return true;
        }
        if (mIsCalibrated) {
            res.success = false;
            res.message = std::format("{} already calibrated", mControllerName);
            return true;
        }
        // sends throttle command until a limit switch is hit
        // mIsCalibrated is set with CAN message coming from BDCMC
        setDesiredThrottle(mCalibrationThrottle);
        res.success = true;
        return true;
    }

} // namespace mrover
