#pragma once

#include <units.hpp>

namespace mrover {

    struct BrushedMotorWriter {
    public:
        void write_output(Volts output);
    private:
        void init();
        bool initialized{false};
    };

} // namespace mrover

