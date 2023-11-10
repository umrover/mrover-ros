#pragma once

#include "units/units.hpp"

#include "messaging.hpp"

namespace mrover {

    struct Config {

        Dimensionless gear_ratio;
        // TODO: Terrible naming for the limit switch info
        ConfigLimitSwitchInfo limit_switch_info;
        ConfigEncoderInfo quad_abs_enc_info;
        Ratio quad_enc_out_ratio;
        Ratio abs_enc_out_ratio;
        Percent max_pwm;
        Radians max_forward_pos;
        Radians max_backward_pos;

        // This stuff is not part of motor message
        std::uint32_t id{};
        bool is_configured{};

        void configure(ConfigCommand const& command) {
            // Flag configuration as initialized
            is_configured = true;

            // Initialize values
            gear_ratio = command.gear_ratio;
            // TODO: Terrible naming for the limit switch info
            limit_switch_info = command.limit_switch_info;
            quad_abs_enc_info = command.quad_abs_enc_info;
            quad_enc_out_ratio = command.quad_enc_out_ratio;
            abs_enc_out_ratio = command.abs_enc_out_ratio;
            max_pwm = command.max_pwm;
            max_forward_pos = command.max_forward_pos;
            max_backward_pos = command.max_backward_pos;
        }
    };

} // namespace mrover
