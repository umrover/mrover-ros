#pragma once

#include <optional>
#include <utility>

#include <units/units.hpp>

#include "stm32g4xx_hal.h"

namespace mrover {

    /**
     * A PIDF controller. An output signal is generated based on the error of the current input and the desired input target.
     *
     * P - Proportional    Multiplied by the error
     * I - Integral        Multiplied by the integral of the error. Alleviates steady state error, use with caution.
     * D - Derivative      Multiplied by the derivative of the error
     * F - Feedforward     Multiplied by the target. This is useful for gravity compensation.
     *
     * @tparam InputUnit   Unit of input, usually a sensor reading (for example encoder ticks for an arm)
     * @tparam OutputUnit  Unit of output, usually a motor command (for example voltage for a motor)
     * @tparam TimeUnit    Unit of time
     */
    template<IsUnit InputUnit, IsUnit OutputUnit, IsUnit TimeUnit = Seconds>
    struct PIDF {
    private:
        using TotalError = compound_unit<InputUnit, TimeUnit>;
        // Settings
        compound_unit<OutputUnit, inverse<InputUnit>> m_p{};
        compound_unit<OutputUnit, inverse<InputUnit>, inverse<TimeUnit>> m_i{};
        compound_unit<OutputUnit, inverse<compound_unit<InputUnit, inverse<TimeUnit>>>> m_d{};
        compound_unit<OutputUnit, inverse<InputUnit>> m_ff{};
        InputUnit m_dead_band{};
        std::pair<OutputUnit, OutputUnit> m_output_bound{};
        std::optional<std::pair<InputUnit, InputUnit>> m_input_bound;
        // State
        TotalError m_total_error{};
        InputUnit m_last_error{};
        TimeUnit m_last_time{};

    public:
        /**
         * TODO: documentation
         *
         * @param input     Current value
         * @param target    Desired final value
         * @param dt        Time since last call
         * @return          Output value to control the input to move to the target
         */
        auto calculate(InputUnit input, InputUnit target, TimeUnit dt) -> OutputUnit {
            InputUnit error = target - input;

            if (m_input_bound) {
                auto [in_min, in_max] = m_input_bound.value();
                if (abs(error) > (in_max - in_min) / 2) {
                    if (error > InputUnit{}) {
                        error -= in_max - in_min;
                    } else {
                        error += in_max - in_min;
                    }
                }
            }

            auto [out_min, out_max] = m_output_bound;
            if (out_min < error * m_p && error * m_p < out_max) {
                m_total_error += error * dt;
            } else {
                m_total_error = TotalError{};
            }

            InputUnit error_for_p = abs(error) < m_dead_band ? InputUnit{} : error;
            OutputUnit result = m_p * error_for_p + m_i * m_total_error + m_d * (error - m_last_error) / dt + m_ff * target;
            m_last_error = error;

            return clamp(result, out_min, out_max);
        }

        auto with_p(double p) -> PIDF& {
            m_p = decltype(m_p){p};
            return *this;
        }

        auto with_i(double i) -> PIDF& {
            m_i = decltype(m_i){i};
            return *this;
        }

        auto with_d(double d) -> PIDF& {
            m_d = decltype(m_d){d};
            return *this;
        }

        auto with_ff(double ff) -> PIDF& {
            m_ff = decltype(m_ff){ff};
            return *this;
        }

        auto with_dead_band(InputUnit dead_band) -> PIDF& {
            m_dead_band = dead_band;
            return *this;
        }

        auto with_input_bound(InputUnit min, InputUnit max) -> PIDF& {
            m_input_bound = {min, max};
            return *this;
        }

        auto with_output_bound(OutputUnit min, OutputUnit max) -> PIDF& {
            m_output_bound = {min, max};
            return *this;
        }
    };

} // namespace mrover
