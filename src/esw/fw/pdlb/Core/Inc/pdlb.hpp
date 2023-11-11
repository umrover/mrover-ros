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
        ADCSensor* adc;
        DiagCurrentSensor* current_sensors[NUM_CURRENT_SENSORS];
        DiagTempSensor* temp_sensors[NUM_TEMP_SENSORS];

        void feed(ArmLaserCommand const& message) {
            // TODO - this needs to be implemented!
        }

        void feed(LEDCommand const& message) {
            // TODO - this needs to be implemented
        }

    public:
        PDLB() = default;

        PDLB(FDCANBus const& fdcan_bus) :
           m_fdcan_bus{fdcan_bus} {

		   adc = new_adc_sensor(&hadc1, NUM_CHANNELS);

		   // Create current sensor objects (test_ADC channels 0-4)
		   for(int i = 0; i < NUM_CURRENT_SENSORS; ++i) {
			  current_sensors[i] = new_diag_current_sensor(adc, i);
		   }

		   // Create temp sensor objects (test_ADC channels 5-9)
		   for(int i = 0; i < NUM_TEMP_SENSORS; ++i) {
			  temp_sensors[i] = new_diag_temp_sensor(adc, i + 5);
		   }

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
