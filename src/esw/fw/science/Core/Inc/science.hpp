#pragma once

#include <concepts>
#include <optional>
#include <variant>
#include "messaging.hpp"
#include "units/units.hpp"
#include "spectral.hpp"
#include "hardware.hpp"
#include "heater.hpp"
#include "cmsis_os2.h"
#include "hardware_adc.hpp"
#include "hardware_i2c.hpp"


namespace mrover {

    class Science {
    private:

    	std::shared_ptr<SMBus<uint8_t, uint8_t>> m_i2c_bus;

    	FDCAN<InBoundScienceMessage> m_fdcan_bus;
        std::array<Spectral, 3> m_spectral_sensors;
        std::shared_ptr<ADCSensor> m_adc_sensor;
        std::array<Heater, 6> m_heaters;
        std::array<Pin, 3> m_uv_leds;
        std::array<Pin, 3> m_white_leds;
        bool m_heater_auto_shutoff_enabled {};
        osMutexId_t m_can_tx_mutex;

        void feed(EnableScienceDeviceCommand const& message) {
        	switch (message.science_device) {
        	case ScienceDevice::HEATER_B0:
        		m_heaters.at(0).enable_if_possible(message.enable);
        		break;
        	case ScienceDevice::HEATER_N0:
				m_heaters.at(1).enable_if_possible(message.enable);
				break;
        	case ScienceDevice::HEATER_B1:
				m_heaters.at(2).enable_if_possible(message.enable);
				break;
        	case ScienceDevice::HEATER_N1:
				m_heaters.at(3).enable_if_possible(message.enable);
				break;
        	case ScienceDevice::HEATER_B2:
				m_heaters.at(4).enable_if_possible(message.enable);
				break;
        	case ScienceDevice::HEATER_N2:
				m_heaters.at(5).enable_if_possible(message.enable);
				break;
        	case ScienceDevice::WHITE_LED_0:
        		m_white_leds.at(0).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
			case ScienceDevice::WHITE_LED_1:
        		m_white_leds.at(1).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
			case ScienceDevice::WHITE_LED_2:
        		m_white_leds.at(2).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
			case ScienceDevice::UV_LED_0:
				m_uv_leds.at(0).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
			case ScienceDevice::UV_LED_1:
				m_uv_leds.at(1).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
			case ScienceDevice::UV_LED_2:
				m_uv_leds.at(2).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
				break;
        	}
        }

        void feed(HeaterAutoShutOffCommand const& message) {
        	for (int i = 0; i < 6; ++i) {
        		m_heaters.at(i).set_auto_shutoff(message.enable_auto_shutoff);
        	}
        }

    public:
        Science() = default;

        Science(FDCAN<InBoundScienceMessage> const& fdcan_bus,
        		std::array<Spectral, 3> spectral_sensors,
				std::shared_ptr<ADCSensor> adc_sensor,
				std::array<DiagTempSensor, 6> diag_temp_sensors,
				std::array<Pin, 6> heater_pins,
				std::array<Pin, 3> uv_leds,
				std::array<Pin, 3> white_leds
				) :
           m_fdcan_bus{fdcan_bus},
		   m_spectral_sensors{std::move(spectral_sensors)},
		   m_adc_sensor{std::move(adc_sensor)},
		   m_uv_leds{std::move(uv_leds)},
		   m_white_leds{std::move(white_leds)},
		   m_can_tx_mutex{osMutexNew(NULL)}
	   {
		   for (int i = 0; i < 6; ++i) {
			   m_heaters.at(i) = Heater(diag_temp_sensors[i], heater_pins[i]);
		   }
	   }

        void receive(InBoundScienceMessage const& message) {
            std::visit([&](auto const& command) { feed(command); }, message);
        }

        void reboot_i2c() {
        	//m_spectral_sensors.at(0).reboot();
        	m_i2c_bus->reboot();
        }

        void update_and_send_spectral() {
        	SpectralData spectral_data;
        	for (int i = 0; i < 3; ++i) {
        		bool op_res = m_spectral_sensors.at(i).update_channel_data();
        		if (!op_res){
        			//TODO: add error handling (whatever should have been in catch) here
        		}




				//spectral_data.spectrals.at(i).error =
				//		m_spectral_sensors.at(i).is_error();
        		for (int j = 0; j < 6; ++j) {
					spectral_data.spectrals.at(i).data.at(j) =
						m_spectral_sensors.at(i).get_channel_data(j);
        		}
//        		for (int j = 0; j < 6; ++j) {
//        			m_spectral_sensors.at(i).update_channel_data(j);
//					spectral_data.spectrals.at(i).data.at(j) =
//							m_spectral_sensors.at(i).get_channel_data(j);
//					spectral_data.spectrals.at(i).error =
//							m_spectral_sensors.at(i).is_error();
//        		}
        	}

        	// TODO - MUTEXS ARE BREAKING CODE!!!! IDK WHY - PLEASE FIX
//        	osMutexAcquire(m_can_tx_mutex, osWaitForever);
			m_fdcan_bus.broadcast(OutBoundScienceMessage{spectral_data});
//			osMutexRelease(m_can_tx_mutex);
        }

        void update_and_send_thermistor_and_auto_shutoff_if_applicable() {
        	m_adc_sensor->update();
        	ThermistorData thermistor_data;

			for(int i = 0; i < 6; ++i) {
				m_heaters.at(i).update_temp_and_auto_shutoff_if_applicable();
				thermistor_data.temps.at(i) = m_heaters.at(i).get_temp();
			}


			/* send current and temperature over CAN */
//			osMutexAcquire(m_can_tx_mutex, osWaitForever);
			m_fdcan_bus.broadcast(OutBoundScienceMessage{thermistor_data});
//			osMutexRelease(m_can_tx_mutex);
        }

        void update_and_send_heater() {

        	for (int i = 0; i < 6; ++i) {
        		m_heaters.at(i).turn_off_if_watchdog_not_fed();
        	}

        	HeaterStateData heater_state_data;

        	for (int i = 0; i < 6; ++i) {
        		SET_BIT_AT_INDEX(heater_state_data.heater_state_info.on, i, m_heaters.at(i).get_state());
        	}

//        	osMutexAcquire(m_can_tx_mutex, osWaitForever);
			m_fdcan_bus.broadcast(OutBoundScienceMessage{heater_state_data});
//			osMutexRelease(m_can_tx_mutex);
        }

    };

} // namespace mrover
