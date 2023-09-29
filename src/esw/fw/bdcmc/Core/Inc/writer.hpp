#pragma once

#include <units.hpp>
#include <config.hpp>

namespace mrover {

    struct BrushedMotorWriter {
    public:
        void write_output(const Config& config, Volts output);
    private:
        void init();
        bool initialized{false};
        bool forward_pin;
        bool backward_pin;
        inline void set_direction_pins(double duty_cycle);
        inline void set_pwm(double duty_cycle);
    };

} // namespace mrover

