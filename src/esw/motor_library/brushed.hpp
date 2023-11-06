#pragma once

#include <can_manager.hpp>
#include <controller.hpp>

#include "messaging.hpp"

namespace mrover {

    class BrushedController : public Controller {
    public:
        void set_desired_throttle(Percent throttle) override; // from -1.0 to 1.0
        void set_desired_velocity(RadiansPerSecond velocity) override;
        void set_desired_position(Radians position) override;

        void send_configuration();

        BrushedController(ros::NodeHandle& nh, const std::string& name) : Controller(nh, name) {
            m_is_configured = false;

            XmlRpc::XmlRpcValue brushed_motor_data;
            nh.getParam("brushed_motors/controllers/" + name, brushed_motor_data);
            assert(nh.hasParam("brushed_motors/controllers/" + name));
            assert(brushed_motor_data.getType() == XmlRpc::XmlRpcValue::TypeArray);

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
            auto driver_voltage = static_cast<double>(brushed_motor_data["gear_ratio"]);
            assert(driver_voltage > 0);
            assert(brushed_motor_data.hasMember("motor_max_voltage") &&
                   brushed_motor_data["motor_max_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            auto motor_max_voltage = static_cast<double>(brushed_motor_data["motor_max_voltage"]);
            assert(0 < motor_max_voltage && motor_max_voltage <= driver_voltage);

            m_config_command.max_pwm = motor_max_voltage / driver_voltage;
            m_config_command.limit_max_pos.limit_max_forward_position = static_cast<bool>(brushed_motor_data["limit_max_forward_pos"]);
            m_config_command.limit_max_pos.limit_max_backward_position = static_cast<bool>(brushed_motor_data["limit_max_backward_pos"]);
            m_config_command.max_forward_pos = Meters{static_cast<double>(brushed_motor_data["limit_max_forward_pos"])};
            m_config_command.max_back_pos = Meters{static_cast<double>(brushed_motor_data["limit_max_backward_pos"])};
        }
        ~BrushedController() override = default;

    private:
        bool m_is_configured;
        ConfigCommand m_config_command;
    };

} // namespace mrover
