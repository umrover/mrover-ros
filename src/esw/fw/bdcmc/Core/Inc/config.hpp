#pragma once

#include <units.hpp>

#include "messaging.hpp"

namespace mrover {

    class Config {
    private:
        uint32_t motorId{};
        Meters gear_ratio{};
        // TODO: Terrible naming for the limit switch info
        uint8_t limit_switch_info_x{};
        uint8_t limit_switch_info_y{};
        uint8_t limit_switch_info_z{};
        uint8_t quad_abs_enc_info{};
        Meters   limit_a_readj_pos{};
        Meters   limit_b_readj_pos{};
        Meters   limit_c_readj_pos{};
        Meters   limit_d_readj_pos{};
        Meters   quad_enc_out_ratio{};
        Meters   abs_enc_out_ratio{};
        Dimensionless   max_pwm{};
        uint8_t limit_max_pos{};
        Meters   max_forward_pos{};
        Meters   max_back_pos{};
        bool isConfigured{false};
    public:

        explicit Config(uint32_t motorId) : motorId{motorId} {}

        void configure(ConfigCommand command) {
            // Flag configuration as initialized
            this->isConfigured = true;

            // Initialize values
            this->gear_ratio = command.gear_ratio;
            // TODO: Terrible naming for the limit switch info
            this->limit_switch_info_x = command.limit_switch_info_x;
            this->limit_switch_info_y = command.limit_switch_info_y;
            this->limit_switch_info_z = command.limit_switch_info_z;
            this->quad_abs_enc_info = command.quad_abs_enc_info;
            this->limit_a_readj_pos = command.limit_a_readj_pos;
            this->limit_b_readj_pos = command.limit_b_readj_pos;
            this->limit_c_readj_pos = command.limit_c_readj_pos;
            this->limit_d_readj_pos = command.limit_d_readj_pos;
            this->quad_enc_out_ratio = command.quad_enc_out_ratio;
            this->abs_enc_out_ratio = command.abs_enc_out_ratio;
            this->max_pwm = command.max_pwm;
            this->limit_max_pos = command.limit_max_pos;
            this->max_forward_pos = command.max_forward_pos;
            this->max_back_pos = command.max_back_pos;
        }

        [[nodiscard]] uint32_t getMotorId() {
            return motorId;
        }

        [[nodiscard]] bool configured() const {
            return this->isConfigured;
        }

        [[nodiscard]] Dimensionless getMaxPWM() const {
            return this->max_pwm;
        }

    };

}
