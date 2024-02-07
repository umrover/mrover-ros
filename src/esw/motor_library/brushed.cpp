#include "brushed.hpp"

namespace mrover {

    BrushedController::BrushedController(ros::NodeHandle const& nh, std::string name, std::string controllerName)
        : Controller{nh, std::move(name), std::move(controllerName)} {

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

        mConfigCommand.quad_abs_enc_info.quad_present = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "quad_present", false);
        mConfigCommand.quad_abs_enc_info.quad_is_forward_polarity = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "quad_is_fwd_polarity", false);
        mConfigCommand.quad_abs_enc_info.abs_present = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "abs_present", false);

        mConfigCommand.quad_abs_enc_info.abs_is_forward_polarity = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "abs_is_fwd_polarity", false);

        mConfigCommand.quad_enc_out_ratio = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "quad_ratio", 1.0);

        mConfigCommand.abs_enc_out_ratio = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "abs_ratio", 1.0);

        auto driver_voltage = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "driver_voltage");
        assert(driver_voltage > 0);
        auto motor_max_voltage = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "motor_max_voltage");
        assert(0 < motor_max_voltage && motor_max_voltage <= driver_voltage);

        mConfigCommand.max_pwm = motor_max_voltage / driver_voltage;

        mConfigCommand.limit_switch_info.limit_max_forward_position = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "limit_max_forward_pos", false);

        mConfigCommand.limit_switch_info.limit_max_backward_position = xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, "limit_max_backward_pos", false);

        mConfigCommand.max_forward_pos = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_forward_pos", 0.0)};
        mConfigCommand.max_backward_pos = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_backward_pos", 0.0)};

        mMinVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "min_velocity", -1.0)};
        mMaxVelocity = RadiansPerSecond{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_velocity", 1.0)};

        mMinPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "min_position", -1.0)};
        mMaxPosition = Radians{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "max_position", 1.0)};

        mPositionGains.p = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_p", 0.0);
        mPositionGains.i = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_i", 0.0);
        mPositionGains.d = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_d", 0.0);
        mPositionGains.ff = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "position_ff", 0.0);

        mVelocityGains.p = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_p", 0.0);
        mVelocityGains.i = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_i", 0.0);
        mVelocityGains.d = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_d", 0.0);
        mVelocityGains.ff = xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "velocity_ff", 0.0);

        for (int i = 0; i < 4; ++i) {
            mhasLimit |= xmlRpcValueToTypeOrDefault<bool>(brushedMotorData, std::format("limit_{}_enabled", i), false);
        }
        mCalibrationThrottle = Percent{xmlRpcValueToTypeOrDefault<double>(brushedMotorData, "calibration_throttle", 0.0)};
        mErrorState = "Unknown";
        mState = "Unknown";
    }

    void BrushedController::setDesiredThrottle(Percent throttle) {
        updateLastConnection();
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        mDevice.publish_message(InBoundMessage{ThrottleCommand{.throttle = throttle}});
    }

    void BrushedController::setDesiredPosition(Radians position) {
        updateLastConnection();
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mMinPosition && position <= mMaxPosition);

        mDevice.publish_message(InBoundMessage{PositionCommand{
                .position = position,
                .p = static_cast<float>(mPositionGains.p),
                .i = static_cast<float>(mPositionGains.i),
                .d = static_cast<float>(mPositionGains.d),
        }});
    }

    void BrushedController::setDesiredVelocity(RadiansPerSecond velocity) {
        updateLastConnection();
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(velocity >= mMinVelocity && velocity <= mMaxVelocity);

        mDevice.publish_message(InBoundMessage{VelocityCommand{
                .velocity = velocity,
                .p = static_cast<float>(mVelocityGains.p),
                .i = static_cast<float>(mVelocityGains.i),
                .d = static_cast<float>(mVelocityGains.d),
                .ff = static_cast<float>(mVelocityGains.ff),
        }});
    }

    void BrushedController::sendConfiguration() {
        mDevice.publish_message(InBoundMessage{mConfigCommand});

        // Need to await configuration. Can NOT directly set mIsConfigured to true.
    }

    void BrushedController::adjust(Radians commandedPosition) {
        updateLastConnection();
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(commandedPosition >= mMinPosition && commandedPosition <= mMaxPosition);

        mDevice.publish_message(InBoundMessage{AdjustCommand{
                .position = commandedPosition
        }});
    }

    void BrushedController::processMessage(ControllerDataState const& state) {
        mCurrentPosition = state.position;
        mCurrentVelocity = state.velocity;
        ROS_INFO("Vel: %f | Pos: %f", mCurrentVelocity.get(), mCurrentPosition.get());
        ConfigCalibErrorInfo configCalibErrInfo = state.config_calib_error_data;
        mIsConfigured = configCalibErrInfo.configured;
        mIsCalibrated = configCalibErrInfo.calibrated;
        mErrorState = errorToString(static_cast<BDCMCErrorInfo>(configCalibErrInfo.error));
        LimitStateInfo limitSStateInfo = state.limit_switches;
        for (std::size_t i = 0; i < mLimitHit.size(); ++i) {
            mLimitHit.at(i) = GET_BIT_AT_INDEX(limitSStateInfo.hit, i);
        }
        if (mIsCalibrated) {
            mState = "Armed";
        } else {
            mState = "Not Armed";
        }
    }

    void BrushedController::processCANMessage(CAN::ConstPtr const& msg) {
        assert(msg->source == mControllerName);
        assert(msg->destination == mName);

        OutBoundMessage const& message = *reinterpret_cast<OutBoundMessage const*>(msg->data.data());

        // This calls the correct process function based on the current value of the alternative
        std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
    }

    double BrushedController::getEffort() {
        return std::numeric_limits<double>::quiet_NaN();
    }

    std::string BrushedController::errorToString(BDCMCErrorInfo errorCode) {
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

    bool BrushedController::calibrateServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (!mhasLimit) {
            res.success = false;
            res.message = mControllerName + " does not have limit switches, cannot calibrate";
            return true;
        } else if (mIsCalibrated) {
            res.success = false;
            res.message = mControllerName + " already calibrated";
            return true;
        } else {
            // sends throttle command until a limit switch is hit
            // mIsCalibrated is set with CAN message coming from BDCMC
            setDesiredThrottle(mCalibrationThrottle); 
            res.success = true;
            return true;
        }
    }

} // namespace mrover
