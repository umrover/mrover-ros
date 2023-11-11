#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "hardware.hpp"
#include "auton_led.hpp"
#include "curr_sensor.hpp"
#include "diag_temp_sensor.hpp"
#include "messaging.hpp"
#include "units.hpp"

namespace mrover {

    class PDLB {
    private:

        FDCANBus m_fdcan_bus;
        Pin m_arm_laser_pin;
        AutonLed m_auton_led;
        std::shared_ptr<ADCSensor> m_adc_sensor_1;
        std::shared_ptr<ADCSensor> m_adc_sensor_2;
        std::array<CurrentSensor, 6> m_current_sensors;
        std::array<DiagTempSensor, 6> m_diag_temp_sensors;
        osMutexId_t m_can_tx_mutex;

        void feed(ArmLaserCommand const& message) {
            m_arm_laser_pin.write(message.enable);
        }

        void feed(LEDCommand const& message) {
        	m_auton_led.change_state(message.led_info.red,
        			message.led_info.green,
					message.led_info.blue,
					message.led_info.blinking);
        }

    public:
        PDLB() = default;

        PDLB(FDCANBus const& fdcan_bus,
        		Pin arm_laser_pin,
        		AutonLed auton_led,
				std::shared_ptr<ADCSensor> adc_sensor_1,
				std::shared_ptr<ADCSensor> adc_sensor_2,
				std::array<CurrentSensor, 6> current_sensors,
				std::array<DiagTempSensor, 6> diag_temp_sensors
				) :
           m_fdcan_bus{fdcan_bus},
		   m_arm_laser_pin{std::move(arm_laser_pin)},
		   m_auton_led{std::move(auton_led)},
		   m_adc_sensor_1{std::move(adc_sensor_1)},
		   m_adc_sensor_2{std::move(adc_sensor_2)},
		   m_current_sensors{std::move(current_sensors)},
		   m_diag_temp_sensors{std::move(diag_temp_sensors)},
		   m_can_tx_mutex{osMutexNew(NULL)}
	   {
	   }

        void receive(InBoundMessage const& message) {
            std::visit([&](auto const& command) { process(command); }, message);
        }

        void update_and_send_current_temp() {
        	m_adc_sensor_1->update();
        	m_adc_sensor_2->update();

			PDBData pdb_data;

			for(int i = 0; i < 6; ++i) {
				m_current_sensors.at(i).update_current();
				pdb_data.currents.at(i) = m_current_sensors.at(i).get_current();
			}

			for(int i = 0; i < NUM_TEMP_SENSORS; ++i) {
				update_diag_temp_sensor_val(temp_sensors[i]);
				m_diag_temp_sensors.at(i).update_temp();
				pdb_data.temperatures.at(i) = m_diag_temp_sensors.at(i).get_temp();
			}

			/* send current and temperature over CAN */
			osMutexAcquire(m_can_tx_mutex, osWaitForever);
			m_fdcan_bus.broadcast(OutBoundPDLBMessage{pdb_data});
			osMutexRelease(m_can_tx_mutex);
        }

        void blink_led_if_applicable() {
        	m_auton_led.blink();
        }

        void receive_message() {
        	if (std::optional received = fdcan_bus.receive<InBoundMessage>()) {
				auto const& [header, message] = received.value();
				auto messageId = std::bit_cast<FDCANBus::MessageId>(header.Identifier);
				if (messageId.destination == DEVICE_ID)
					pdlb.receive(message);
			}
        }
    };

} // namespace mrover
