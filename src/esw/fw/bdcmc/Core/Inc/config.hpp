#pragma once

#include <units.hpp>

#include "messaging.hpp"

namespace mrover {

    struct Config {

        Dimensionless gear_ratio;
        // TODO: Terrible naming for the limit switch info
        ConfigLimitSwitchInfo0 limit_switch_info_0;
        ConfigLimitSwitchInfo1 limit_switch_info_1;
        ConfigLimitSwitchInfo2 limit_switch_info_2;
        ConfigEncoderInfo quad_abs_enc_info;
        Radians limit_a_readj_pos;
        Radians limit_b_readj_pos;
        Radians limit_c_readj_pos;
        Radians limit_d_readj_pos;
        Ratio quad_enc_out_ratio;
        Ratio abs_enc_out_ratio;
        Percent max_pwm;
        ConfigLimitInfo limit_max_pos;
        Meters max_forward_pos;
        Meters max_back_pos;

        // This stuff is not part of motor message
        std::uint32_t id{};
        bool is_configured{};

        void configure(ConfigCommand const& command) {
            // Flag configuration as initialized
            this->is_configured = true;

            // Initialize values
            this->gear_ratio = command.gear_ratio;
            // TODO: Terrible naming for the limit switch info
            // TODO(guthrie): make this compile
            //            this->limit_switch_info_x = command.limit_switch_info_x;
            //            this->limit_switch_info_y = command.limit_switch_info_y;
            //            this->limit_switch_info_z = command.limit_switch_info_z;
            //            this->quad_abs_enc_info = command.quad_abs_enc_info;
            //            this->limit_a_readj_pos = command.limit_a_readj_pos;
            //            this->limit_b_readj_pos = command.limit_b_readj_pos;
            //            this->limit_c_readj_pos = command.limit_c_readj_pos;
            //            this->limit_d_readj_pos = command.limit_d_readj_pos;
            //            this->quad_enc_out_ratio = command.quad_enc_out_ratio;
            //            this->abs_enc_out_ratio = command.abs_enc_out_ratio;
            //            this->max_duty_cycle = command.max_pwm;
            //            this->limit_max_pos = command.limit_max_pos;
            //            this->max_forward_pos = command.max_forward_pos;
            //            this->max_back_pos = command.max_back_pos;
        }
    };

} // namespace mrover
