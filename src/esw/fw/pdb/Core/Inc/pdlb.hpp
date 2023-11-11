#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "hardware.hpp"
#include "messaging.hpp"
#include "units.hpp"

namespace mrover {

    class PDLB {
    private:

    	// TODO - Add Celsius to the units library
    	PDBData pdb_data;

        FDCANBus m_fdcan_bus;

        void feed(ArmLaserCommand const& message) {
            // TODO - this needs to be implemented!
        }

        void feed(LEDCommand const& message) {
            // TODO - this needs to be implemented
        }

    public:
        PDLB() = default;

        PDLB(FDCANBus const& fdcan_bus) :
           m_fdcan_bus{fdcan_bus} {}

        void receive(InBoundMessage const& message) {
            std::visit([&](auto const& command) { process(command); }, message);
        }

        void update_and_send_current_temp() {
        	// TODO - implement
        }

        void update_led() {
        	// TODO - implement and call inside main
        }

        void receive_messages() {
        	if (std::optional received = fdcan_bus.receive<InBoundMessage>()) {
				auto const& [header, message] = received.value();
				auto messageId = std::bit_cast<FDCANBus::MessageId>(header.Identifier);
				if (messageId.destination == DEVICE_ID)
					controller.receive(message);
			}
        }
    };

} // namespace mrover
