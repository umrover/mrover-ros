#include <science.hpp>
#include <cstdint>
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

namespace mrover {

    // NOTE: Change this for the PDLB controller
    constexpr static std::uint8_t DEVICE_ID = 0x32;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCAN fdcan_bus;
    Science science;

    void init() {
        check(HAL_FDCAN_ActivateNotification(
                      &hfdcan1,
                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                      0) == HAL_OK,
              Error_Handler);

        std::shared_ptr<SMBus> i2c_bus = std::make_shared<SMBus>(&hi2c1);

        std::shared_ptr<I2CMux> i2c_mux = std::make_shared<I2CMux>(i2c_bus);

        std::array<Spectral, 3> spectral_sensors = {
        		Spectral(i2c_bus, i2c_mux, 0),
				Spectral(i2c_bus, i2c_mux, 1),
				Spectral(i2c_bus, i2c_mux, 2)
        };

        std::shared_ptr<ADCSensor> adc_sensor = std::make_shared<ADCSensor>(&hadc1, 6);

        std::array<DiagTempSensor, 6> diag_temp_sensors =
		{
				DiagTempSensor{adc_sensor, 0},
				DiagTempSensor{adc_sensor, 1},
				DiagTempSensor{adc_sensor, 2},
				DiagTempSensor{adc_sensor, 3},
				DiagTempSensor{adc_sensor, 4},
				DiagTempSensor{adc_sensor, 5},

		};
        std::array<Pin, 6> heater_pins =
        {
        		Pin{HEATER_B0_GPIO_Port, HEATER_B0_Pin},
				Pin{HEATER_N0_GPIO_Port, HEATER_N0_Pin},
				Pin{HEATER_B1_GPIO_Port, HEATER_B1_Pin},
				Pin{HEATER_N1_GPIO_Port, HEATER_N1_Pin},
				Pin{HEATER_B2_GPIO_Port, HEATER_B2_Pin},
				Pin{HEATER_N2_GPIO_Port, HEATER_N2_Pin}
        };
        std::array<Pin, 3> uv_leds =
		{
				Pin{UV_LED_0_GPIO_Port, UV_LED_0_Pin},
				Pin{UV_LED_1_GPIO_Port, UV_LED_1_Pin},
				Pin{UV_LED_1_GPIO_Port, UV_LED_2_Pin}
		};
        std::array<Pin, 3> white_leds =
		{
				Pin{WHITE_LED_0_GPIO_Port, WHITE_LED_0_Pin},
				Pin{WHITE_LED_1_GPIO_Port, WHITE_LED_1_Pin},
				Pin{WHITE_LED_1_GPIO_Port, WHITE_LED_2_Pin}
		};

        fdcan_bus = FDCAN{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        science = Science{fdcan_bus, spectral_sensors, adc_sensor, diag_temp_sensors, heater_pins, uv_leds, white_leds};
    }

    void update_and_send_spectral() {
    	science.update_and_send_spectral();
    }

    void update_and_send_thermistor_and_auto_shutoff_if_applicable() {
		science.update_and_send_thermistor_and_auto_shutoff_if_applicable();
	}

    void update_and_send_heater() {
		science.update_and_send_heater();
	}

    void receive_message() {
		if (std::optional received = fdcan_bus.receive<InBoundScienceMessage>()) {
			auto const& [header, message] = received.value();
			auto messageId = std::bit_cast<FDCAN::MessageId>(header.Identifier);
			if (messageId.destination == DEVICE_ID)
				science.receive(message);
		}
	}

} // namespace mrover

void init() {
    mrover::init();
}

void update_and_send_spectral() {
	mrover::update_and_send_spectral();
}

void update_and_send_thermistor_and_auto_shutoff_if_applicable() {
	mrover::update_and_send_thermistor_and_auto_shutoff_if_applicable();
}

void update_and_send_heater() {
	mrover::update_and_send_heater();
}

void receive_message() {
	mrover::receive_message();
}
