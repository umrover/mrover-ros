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

    	FDCAN<InBoundPDLBMessage> m_fdcan_bus;
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
           m_fdcan_bus{fdcan_bus},
		   m_arm_laser_pin{std::move(arm_laser_pin)},
		   m_auton_led{std::move(auton_led)}
//		   m_can_tx_mutex{osMutexNew(NULL)}
		   {
		   }

        void receive(InBoundPDLBMessage const& message) {
            std::visit([&](auto const& command) { feed(command); }, message);
        }



        void blink_led_if_applicable() {
        	m_auton_led.blink();
        }
    };

} // namespace mrover
