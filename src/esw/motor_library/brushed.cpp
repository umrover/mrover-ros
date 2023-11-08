#include "brushed.hpp"

namespace mrover {

    const std::unordered_map<int, std::string> BrushedController::mCodeToError = {
            {0, "No Error"},
            {1, "Error"}}; // TODO - NEED MORE SPECIFIC ERRORS

    BrushedController::BrushedController(ros::NodeHandle const& nh, std::string name, std::string controllerName)
        : Controller{nh, std::move(name), std::move(controllerName)} {

        XmlRpc::XmlRpcValue brushedMotorData;
        mNh.getParam(std::format("brushed_motors/controllers/{}", mControllerName), brushedMotorData);
        assert(mNh.hasParam(std::format("brushed_motors/controllers/{}", mControllerName)));
        assert(brushedMotorData.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        assert(brushedMotorData.hasMember("gear_ratio") &&
               brushedMotorData["gear_ratio"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        mConfigCommand.gear_ratio = static_cast<double>(brushedMotorData["gear_ratio"]);

        mConfigCommand.limit_switch_info_0.a_present = static_cast<bool>(brushedMotorData["limit_a_present"]);
        mConfigCommand.limit_switch_info_0.b_present = static_cast<bool>(brushedMotorData["limit_b_present"]);
        mConfigCommand.limit_switch_info_0.c_present = static_cast<bool>(brushedMotorData["limit_c_present"]);
        mConfigCommand.limit_switch_info_0.d_present = static_cast<bool>(brushedMotorData["limit_d_present"]);
        mConfigCommand.limit_switch_info_0.a_enable = static_cast<bool>(brushedMotorData["limit_a_enable"]);
        mConfigCommand.limit_switch_info_0.b_enable = static_cast<bool>(brushedMotorData["limit_b_enable"]);
        mConfigCommand.limit_switch_info_0.c_enable = static_cast<bool>(brushedMotorData["limit_c_enable"]);
        mConfigCommand.limit_switch_info_0.d_enable = static_cast<bool>(brushedMotorData["limit_d_enable"]);

        mConfigCommand.limit_switch_info_1.a_active_high = static_cast<bool>(brushedMotorData["limit_a_is_active_high"]);
        mConfigCommand.limit_switch_info_1.b_active_high = static_cast<bool>(brushedMotorData["limit_b_is_active_high"]);
        mConfigCommand.limit_switch_info_1.c_active_high = static_cast<bool>(brushedMotorData["limit_c_is_active_high"]);
        mConfigCommand.limit_switch_info_1.d_active_high = static_cast<bool>(brushedMotorData["limit_d_is_active_high"]);
        mConfigCommand.limit_switch_info_1.a_limits_forward = static_cast<bool>(brushedMotorData["limit_a_limits_fwd"]);
        mConfigCommand.limit_switch_info_1.b_limits_forward = static_cast<bool>(brushedMotorData["limit_b_limits_fwd"]);
        mConfigCommand.limit_switch_info_1.c_limits_forward = static_cast<bool>(brushedMotorData["limit_c_limits_fwd"]);
        mConfigCommand.limit_switch_info_1.d_limits_forward = static_cast<bool>(brushedMotorData["limit_d_limits_fwd"]);

        mConfigCommand.limit_switch_info_2.a_use_for_readjustment = static_cast<bool>(brushedMotorData["limit_a_used_for_readjustment"]);
        mConfigCommand.limit_switch_info_2.b_use_for_readjustment = static_cast<bool>(brushedMotorData["limit_b_used_for_readjustment"]);
        mConfigCommand.limit_switch_info_2.c_use_for_readjustment = static_cast<bool>(brushedMotorData["limit_c_used_for_readjustment"]);
        mConfigCommand.limit_switch_info_2.d_use_for_readjustment = static_cast<bool>(brushedMotorData["limit_d_used_for_readjustment"]);
        mConfigCommand.limit_switch_info_2.a_is_default_enabled = static_cast<bool>(brushedMotorData["limit_a_default_enabled"]);
        mConfigCommand.limit_switch_info_2.b_is_default_enabled = static_cast<bool>(brushedMotorData["limit_b_default_enabled"]);
        mConfigCommand.limit_switch_info_2.c_is_default_enabled = static_cast<bool>(brushedMotorData["limit_c_default_enabled"]);
        mConfigCommand.limit_switch_info_2.d_is_default_enabled = static_cast<bool>(brushedMotorData["limit_d_default_enabled"]);

        mConfigCommand.quad_abs_enc_info.quad_present = static_cast<bool>(brushedMotorData["quad_present"]);
        mConfigCommand.quad_abs_enc_info.quad_is_forward_polarity = static_cast<bool>(brushedMotorData["quad_is_forward_polarity"]);
        mConfigCommand.quad_abs_enc_info.abs_present = static_cast<bool>(brushedMotorData["abs_present"]);
        mConfigCommand.quad_abs_enc_info.abs_is_forward_polarity = static_cast<bool>(brushedMotorData["abs_is_forward_polarity"]);

        mConfigCommand.limit_a_readj_pos = Radians{static_cast<double>(brushedMotorData["limit_a_readjust_position"])};
        mConfigCommand.limit_b_readj_pos = Radians{static_cast<double>(brushedMotorData["limit_b_readjust_position"])};
        mConfigCommand.limit_c_readj_pos = Radians{static_cast<double>(brushedMotorData["limit_c_readjust_position"])};
        mConfigCommand.limit_d_readj_pos = Radians{static_cast<double>(brushedMotorData["limit_d_readjust_position"])};
        mConfigCommand.quad_enc_out_ratio = static_cast<double>(brushedMotorData["quad_ratio"]);
        mConfigCommand.abs_enc_out_ratio = static_cast<double>(brushedMotorData["abs_ratio"]);

        assert(brushedMotorData.hasMember("driver_voltage") &&
               brushedMotorData["driver_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto driver_voltage = static_cast<double>(brushedMotorData["driver_voltage"]);
        assert(driver_voltage > 0);
        assert(brushedMotorData.hasMember("motor_max_voltage") &&
               brushedMotorData["motor_max_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto motor_max_voltage = static_cast<double>(brushedMotorData["motor_max_voltage"]);
        assert(0 < motor_max_voltage && motor_max_voltage <= driver_voltage);

        mConfigCommand.max_pwm = motor_max_voltage / driver_voltage;

        mConfigCommand.limit_max_pos.limit_max_forward_position = static_cast<bool>(brushedMotorData["limit_max_forward_pos"]);
        mConfigCommand.limit_max_pos.limit_max_backward_position = static_cast<bool>(brushedMotorData["limit_max_backward_pos"]);
        mConfigCommand.max_forward_pos = Meters{static_cast<double>(brushedMotorData["max_forward_pos"])};
        mConfigCommand.max_back_pos = Meters{static_cast<double>(brushedMotorData["max_backward_pos"])};

        mErrorState = "Unknown";
        mState = "Unknown";
    }

    void BrushedController::setDesiredThrottle(Percent throttle) {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        mDevice.publish_message(InBoundMessage{ThrottleCommand{.throttle = throttle}});
    }

    void BrushedController::setDesiredPosition(Radians position) {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mMinPosition && position <= mMaxPosition);

        mDevice.publish_message(InBoundMessage{PositionCommand{.position = position}});
    }

    void BrushedController::setDesiredVelocity(RadiansPerSecond velocity) {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(velocity >= mMinVelocity && velocity <= mMaxVelocity);

        mDevice.publish_message(InBoundMessage{VelocityCommand{.velocity = velocity}});
    }

    void BrushedController::sendConfiguration() {
        mDevice.publish_message(InBoundMessage{mConfigCommand});

        // Need to await configuration. Can NOT directly set mIsConfigured to true.
    }

    void BrushedController::processMessage(ControllerDataState const& state) {
        mCurrentPosition = state.position;
        mCurrentVelocity = state.velocity;
        ConfigCalibErrorInfo configCalibErrInfo = state.config_calib_error_data;
        mIsConfigured = configCalibErrInfo.configured;
        mIsCalibrated = configCalibErrInfo.calibrated;
        mErrorState = mCodeToError.at(configCalibErrInfo.error);
        LimitStateInfo limitSStateInfo = state.limit_switches;
        mLimitAHit = limitSStateInfo.limit_a_hit;
        mLimitBHit = limitSStateInfo.limit_b_hit;
        mLimitCHit = limitSStateInfo.limit_c_hit;
        mLimitDHit = limitSStateInfo.limit_d_hit;
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

} // namespace mrover
