#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "hardware.hpp"
#include "auton_led.hpp"
#include "messaging.hpp"
#include "units/units.hpp"

namespace mrover {

    class PDLB {
    private:

    	FDCAN<InBoundPDLBMessage> fdcan_bus;
        Pin m_arm_laser_pin;
        AutonLed m_auton_led;
//        osMutexId_t m_can_tx_mutex;

        void feed(ArmLaserCommand const& message) {
            m_arm_laser_pin.write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        void feed(LEDCommand const& message) {
        	m_auton_led.change_state(message.led_info.red,
        			message.led_info.green,
					message.led_info.blue,
					message.led_info.blinking);
        }

    public:
        PDLB() = default;

        PDLB(FDCAN<InBoundPDLBMessage> const& fdcan_bus,
        		Pin arm_laser_pin,
        		AutonLed auton_led
				) :
           fdcan_bus{fdcan_bus},
		   m_arm_laser_pin{std::move(arm_laser_pin)},
		   m_auton_led{std::move(auton_led)}
//		   m_can_tx_mutex{osMutexNew(NULL)}
		   {
		   }

        void receive(InBoundPDLBMessage const& message) {
            std::visit([&](auto const& command) { feed(command); }, message);
        }



        void blink_led_if_applicable() {
        	PDBData pdb_data;
        	pdb_data.currents = {0};
        	pdb_data.temperatures = {0};
        	m_auton_led.blink();
        	// broadcast message
        	fdcan_bus.broadcast(OutBoundPDLBMessage{pdb_data});
        }
    };

} // namespace mrover
