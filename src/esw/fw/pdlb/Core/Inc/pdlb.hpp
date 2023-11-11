#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "hardware.hpp"
#include "auton_led.hpp"
#include "messaging.hpp"
#include "units.hpp"

namespace mrover {

    class PDLB {
    private:

        FDCANBus m_fdcan_bus;
        ADCSensor* m_adc_1;
        ADCSensor* m_adc_2;
        DiagCurrentSensor* m_current_sensors[NUM_CURRENT_SENSORS];
        DiagTempSensor* m_temp_sensors[NUM_TEMP_SENSORS];
        osMutexId_t m_mutex_id1;
        AutonLed m_auton_led;

        void feed(ArmLaserCommand const& message) {
            // TODO - this needs to be implemented!
        }

        void feed(LEDCommand const& message) {
            // TODO - this needs to be implemented
        }

    public:
        PDLB() = default;

        PDLB(FDCANBus const& fdcan_bus, AutonLed auton_led) :
           m_fdcan_bus{fdcan_bus}, m_auton_led{std::move(auton_led)} {

		   m_adc_1 = new_adc_sensor(&hadc1, 10);
		   m_adc_2 = new_adc_sensor(&hadc2, 2);

		   // TODO - fix - the ioc isnt actually like this
		   for(int i = 0; i < NUM_CURRENT_SENSORS; ++i) {
			  m_current_sensors[i] = new_diag_current_sensor(adc, i);
		   }

		   // Create temp sensor objects (test_ADC channels 5-9)
		   for(int i = 0; i < NUM_TEMP_SENSORS; ++i) {
			  m_temp_sensors[i] = new_diag_temp_sensor(adc, i + 5);
		   }

		   can_tx_mutex = osMutexNew(NULL); // default attr. for now

	   }

        void receive(InBoundMessage const& message) {
            std::visit([&](auto const& command) { process(command); }, message);
        }

        void update_and_send_current_temp() {
			update_adc_sensor_values(adc);

			/* update current and temperature values + build CAN msg */
			PDBData pdb_data;

			for(int i = 0; i < NUM_CURRENT_SENSORS; ++i) {
				update_diag_current_sensor_val(current_sensors[i]);
				pdb_data.currents[i] = get_diag_current_sensor_val(current_sensors[i]);
			}

			for(int i = 0; i < NUM_TEMP_SENSORS; ++i) {
				update_diag_temp_sensor_val(temp_sensors[i]);
				pdb_data.temperatures[i] = get_diag_temp_sensor_val(temp_sensors[i]);
			}

			/* send current and temperature over CAN */
			osMutexAcquire(can_tx_mutex, osWaitForever);
			m_fdcan_bus.broadcast(OutBoundPDLBMessage{pdb_data});
			osMutexRelease(can_tx_mutex);
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
