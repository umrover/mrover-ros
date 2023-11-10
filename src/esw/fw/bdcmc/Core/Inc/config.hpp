#pragma once

#include "units/units.hpp"

#include "messaging.hpp"

namespace mrover {

    struct Config {

        Dimensionless gear_ratio;
        // TODO: Terrible naming for the limit switch info
        ConfigLimitSwitchInfo0 limit_switch_info_0;
        ConfigLimitSwitchInfo1 limit_switch_info_1;
        ConfigLimitSwitchInfo2 limit_switch_info_2;
        ConfigEncoderInfo quad_abs_enc_info;
        std::array<Radians, 4> limit_readj_pos;
        Ratio quad_enc_out_ratio;
        Ratio abs_enc_out_ratio;
        Percent max_pwm;
        ConfigLimitInfo limit_max_pos;
        Radians max_forward_pos;
        Radians max_backward_pos;

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
            this->limit_switch_info_0 = command.limit_switch_info_0;
            this->limit_switch_info_1 = command.limit_switch_info_1;
            this->limit_switch_info_2 = command.limit_switch_info_2;
            this->quad_abs_enc_info = command.quad_abs_enc_info;
            for (std::size_t i = 0; i < this->limit_readj_pos.size(); ++i) {
                this->limit_readj_pos[i] = command.limit_readj_pos[i];
            }
            this->quad_enc_out_ratio = command.quad_enc_out_ratio;
            this->abs_enc_out_ratio = command.abs_enc_out_ratio;
            this->max_pwm = command.max_pwm;
            this->limit_max_pos = command.limit_max_pos;
            this->max_forward_pos = command.max_forward_pos;
            this->max_back_pos = command.max_back_pos;
        }
    };

} // namespace mrover
