#pragma once

#include <units.hpp>

#include "messaging.hpp"

namespace mrover {

    class Config {
    private:
        uint32_t motorId{};
        Volts maxVoltage{};
        bool isConfigured{false};
    public:

        explicit Config(uint32_t motorId) : motorId{motorId} {}

        void configure(ConfigCommand command) {
            // Flag configuration as initialized
            this->isConfigured = true;

            // Initialize values
            this->maxVoltage = command.maxVolts;
        }

        [[nodiscard]] uint32_t getMotorId() {
            return motorId;
        }

        [[nodiscard]] bool configured() const {
            return this->isConfigured;
        }

        [[nodiscard]] Volts getMaxVoltage() const {
            return this->maxVoltage;
        }

    };

}
