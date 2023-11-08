#include "brushed.hpp"

namespace mrover {

    const std::unordered_map<int, std::string> BrushedController::brushedErrorByCode = {
            {0, "No Error"},
            {1, "Error"}}; // TODO - NEED MORE SPECIFIC ERRORS

    BrushedController::BrushedController(ros::NodeHandle const& nh, std::string name, std::string controller_name)
        : Controller{nh, std::move(name), std::move(controller_name)} {

        XmlRpc::XmlRpcValue brushed_motor_data;
        mNh.getParam(std::format("brushed_motors/controllers/{}", mControllerName), brushed_motor_data);
        assert(mNh.hasParam(std::format("brushed_motors/controllers/{}", mControllerName)));
        assert(brushed_motor_data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        assert(brushed_motor_data.hasMember("gear_ratio") &&
               brushed_motor_data["gear_ratio"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        m_config_command.gear_ratio = static_cast<double>(brushed_motor_data["gear_ratio"]);

        m_config_command.limit_switch_info_0.a_present = static_cast<bool>(brushed_motor_data["limit_a_present"]);
        m_config_command.limit_switch_info_0.b_present = static_cast<bool>(brushed_motor_data["limit_b_present"]);
        m_config_command.limit_switch_info_0.c_present = static_cast<bool>(brushed_motor_data["limit_c_present"]);
        m_config_command.limit_switch_info_0.d_present = static_cast<bool>(brushed_motor_data["limit_d_present"]);
        m_config_command.limit_switch_info_0.a_enable = static_cast<bool>(brushed_motor_data["limit_a_enable"]);
        m_config_command.limit_switch_info_0.b_enable = static_cast<bool>(brushed_motor_data["limit_b_enable"]);
        m_config_command.limit_switch_info_0.c_enable = static_cast<bool>(brushed_motor_data["limit_c_enable"]);
        m_config_command.limit_switch_info_0.d_enable = static_cast<bool>(brushed_motor_data["limit_d_enable"]);

        m_config_command.limit_switch_info_1.a_active_high = static_cast<bool>(brushed_motor_data["limit_a_is_active_high"]);
        m_config_command.limit_switch_info_1.b_active_high = static_cast<bool>(brushed_motor_data["limit_b_is_active_high"]);
        m_config_command.limit_switch_info_1.c_active_high = static_cast<bool>(brushed_motor_data["limit_c_is_active_high"]);
        m_config_command.limit_switch_info_1.d_active_high = static_cast<bool>(brushed_motor_data["limit_d_is_active_high"]);
        m_config_command.limit_switch_info_1.a_limits_forward = static_cast<bool>(brushed_motor_data["limit_a_limits_fwd"]);
        m_config_command.limit_switch_info_1.b_limits_forward = static_cast<bool>(brushed_motor_data["limit_b_limits_fwd"]);
        m_config_command.limit_switch_info_1.c_limits_forward = static_cast<bool>(brushed_motor_data["limit_c_limits_fwd"]);
        m_config_command.limit_switch_info_1.d_limits_forward = static_cast<bool>(brushed_motor_data["limit_d_limits_fwd"]);

        m_config_command.limit_switch_info_2.a_use_for_readjustment = static_cast<bool>(brushed_motor_data["limit_a_used_for_readjustment"]);
        m_config_command.limit_switch_info_2.b_use_for_readjustment = static_cast<bool>(brushed_motor_data["limit_b_used_for_readjustment"]);
        m_config_command.limit_switch_info_2.c_use_for_readjustment = static_cast<bool>(brushed_motor_data["limit_c_used_for_readjustment"]);
        m_config_command.limit_switch_info_2.d_use_for_readjustment = static_cast<bool>(brushed_motor_data["limit_d_used_for_readjustment"]);
        m_config_command.limit_switch_info_2.a_is_default_enabled = static_cast<bool>(brushed_motor_data["limit_a_default_enabled"]);
        m_config_command.limit_switch_info_2.b_is_default_enabled = static_cast<bool>(brushed_motor_data["limit_b_default_enabled"]);
        m_config_command.limit_switch_info_2.c_is_default_enabled = static_cast<bool>(brushed_motor_data["limit_c_default_enabled"]);
        m_config_command.limit_switch_info_2.d_is_default_enabled = static_cast<bool>(brushed_motor_data["limit_d_default_enabled"]);

        m_config_command.quad_abs_enc_info.quad_present = static_cast<bool>(brushed_motor_data["quad_present"]);
        m_config_command.quad_abs_enc_info.quad_is_forward_polarity = static_cast<bool>(brushed_motor_data["quad_is_forward_polarity"]);
        m_config_command.quad_abs_enc_info.abs_present = static_cast<bool>(brushed_motor_data["abs_present"]);
        m_config_command.quad_abs_enc_info.abs_is_forward_polarity = static_cast<bool>(brushed_motor_data["abs_is_forward_polarity"]);

        m_config_command.limit_a_readj_pos = Radians{static_cast<double>(brushed_motor_data["limit_a_readjust_position"])};
        m_config_command.limit_b_readj_pos = Radians{static_cast<double>(brushed_motor_data["limit_b_readjust_position"])};
        m_config_command.limit_c_readj_pos = Radians{static_cast<double>(brushed_motor_data["limit_c_readjust_position"])};
        m_config_command.limit_d_readj_pos = Radians{static_cast<double>(brushed_motor_data["limit_d_readjust_position"])};
        m_config_command.quad_enc_out_ratio = static_cast<double>(brushed_motor_data["quad_ratio"]);
        m_config_command.abs_enc_out_ratio = static_cast<double>(brushed_motor_data["abs_ratio"]);

        assert(brushed_motor_data.hasMember("driver_voltage") &&
               brushed_motor_data["driver_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto driver_voltage = static_cast<double>(brushed_motor_data["driver_voltage"]);
        assert(driver_voltage > 0);
        assert(brushed_motor_data.hasMember("motor_max_voltage") &&
               brushed_motor_data["motor_max_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto motor_max_voltage = static_cast<double>(brushed_motor_data["motor_max_voltage"]);
        assert(0 < motor_max_voltage && motor_max_voltage <= driver_voltage);

        m_config_command.max_pwm = motor_max_voltage / driver_voltage;

        m_config_command.limit_max_pos.limit_max_forward_position = static_cast<bool>(brushed_motor_data["limit_max_forward_pos"]);
        m_config_command.limit_max_pos.limit_max_backward_position = static_cast<bool>(brushed_motor_data["limit_max_backward_pos"]);
        m_config_command.max_forward_pos = Meters{static_cast<double>(brushed_motor_data["max_forward_pos"])};
        m_config_command.max_back_pos = Meters{static_cast<double>(brushed_motor_data["max_backward_pos"])};

        mErrorState = "Unknown";
        mState = "Unknown";
    }

    void BrushedController::set_desired_throttle(Percent throttle) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        mDevice.publish_message(InBoundMessage{ThrottleCommand{.throttle = throttle}});
    }

    void BrushedController::set_desired_position(Radians position) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(position >= mMinPosition && position <= mMaxPosition);

        mDevice.publish_message(InBoundMessage{PositionCommand{.position = position}});
    }

    void BrushedController::set_desired_velocity(RadiansPerSecond velocity) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(velocity >= mMinVelocity && velocity <= mMaxVelocity);

        mDevice.publish_message(InBoundMessage{VelocityCommand{.velocity = velocity}});
    }

    void BrushedController::send_configuration() {
        mDevice.publish_message(InBoundMessage{m_config_command});

        // TODO: do we need to await confirmation?
        m_is_configured = true;
    }

    void BrushedController::processCANMessage(CAN::ConstPtr const& msg) {
        assert(msg->source == mControllerName);
        assert(msg->destination == mName);

        auto* out_bound_message = reinterpret_cast<OutBoundMessage const*>(msg->data.data());

        std::visit([&](auto&& message) {
            if constexpr (std::is_same_v<std::decay_t<decltype(message)>, ControllerDataState>) {
                ControllerDataState controller_data_state;
                mCurrentPosition = controller_data_state.position;
                mCurrentVelocity = controller_data_state.velocity;
                ConfigCalibErrorInfo config_calib_err_info = controller_data_state.config_calib_error_data;
                m_is_configured = config_calib_err_info.configured;
                mIsCalibrated = config_calib_err_info.calibrated;
                mErrorState = brushedErrorByCode.at(config_calib_err_info.error);
                LimitStateInfo limit_state_info = controller_data_state.limit_switches;
                mLimitAHit = limit_state_info.limit_a_hit;
                mLimitBHit = limit_state_info.limit_b_hit;
                mLimitCHit = limit_state_info.limit_c_hit;
                mLimitDHit = limit_state_info.limit_d_hit;
                if (mIsCalibrated) {
                    mState = "Armed";
                } else {
                    mState = "Not Armed";
                }
            } else {
                // do whatever
            }
        },
                   out_bound_message);
    }

    double BrushedController::getEffort() {
        return std::nan("1"); // TODO - is there a purpose for the tag
    }

} // namespace mrover
