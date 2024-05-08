#include <science.hpp>
#include <cstdint>
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

extern uint8_t spectral_status_buffer[1];

namespace mrover {

    constexpr static std::uint8_t DEVICE_ID = 0x51;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCAN<InBoundScienceMessage> fdcan_bus;
    Science science;

    void init() {
    	// Instantiation of SMBus provides # of bytes in I2C transactions
        std::shared_ptr<SMBus<uint8_t, uint8_t>> i2c_bus = std::make_shared<SMBus<uint8_t, uint8_t>>(&hi2c1);

        std::shared_ptr<I2CMux> i2c_mux = std::make_shared<I2CMux>(
        		i2c_bus,
				Pin{I2C_MUX_RST_GPIO_Port, I2C_MUX_RST_Pin});

        std::array<Spectral, 3> spectral_sensors = {
        		Spectral(i2c_bus, i2c_mux, 0),
				Spectral(i2c_bus, i2c_mux, 1),
				Spectral(i2c_bus, i2c_mux, 2)
        };

        std::shared_ptr<ADCSensor> adc_sensor = std::make_shared<ADCSensor>(&hadc1, 6);

        std::array<Thermistor, 6> thermistors =
		{
				Thermistor{adc_sensor, 0},
				Thermistor{adc_sensor, 1},
				Thermistor{adc_sensor, 2},
				Thermistor{adc_sensor, 3},
				Thermistor{adc_sensor, 4},
				Thermistor{adc_sensor, 5},

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
				Pin{UV_LED_2_GPIO_Port, UV_LED_2_Pin}
		};
        std::array<Pin, 3> white_leds =
		{
				Pin{WHITE_LED_0_GPIO_Port, WHITE_LED_0_Pin},
				Pin{WHITE_LED_1_GPIO_Port, WHITE_LED_1_Pin},
				Pin{WHITE_LED_2_GPIO_Port, WHITE_LED_2_Pin}
		};

        fdcan_bus = FDCAN<InBoundScienceMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        science = Science{fdcan_bus, spectral_sensors, adc_sensor, thermistors, heater_pins, uv_leds, white_leds};
    }

    void reboot_spectral() {
    	science.reboot_i2c();
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
    	std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundScienceMessage>> received = fdcan_bus.receive();
		if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

		auto const& [header, message] = received.value();

		auto messageId = std::bit_cast<FDCAN<InBoundScienceMessage>::MessageId>(header.Identifier);

		if (messageId.destination == DEVICE_ID) {
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        receive_message();
    } else {
        // Mailbox is full OR we lost a frame
        Error_Handler();
    }
}

//void HAL_I2C_MasterTXCpltCallback(I2C_HandleTypeDef *hi2c) {
//	HAL_I2C_Master_Receive_IT(hi2c, (mrover::Spectral::SPECTRAL_7b_ADDRESS << 1 | 1), (uint8_t*)spectral_status_buffer, sizeof(spectral_status_buffer));
//}
//
//void HAL_I2C_MasterRXCpltCallback(I2C_HandleTypeDef *hi2c) {
//	// If we want to use this for anything else than checking spectral status reg,
//	// then this function needs additional logic
//
//	if ((spectral_status_buffer[0] & mrover::Spectral::I2C_AS72XX_SLAVE_TX_VALID) == 0) {
//		osSemaphoreRelease(spectral_write_status);
//	}
//
//	if ((spectral_status_buffer[0] & mrover::Spectral::I2C_AS72XX_SLAVE_RX_VALID) == 0) {
//		osSemaphoreRelease(spectral_read_status);
//	}
//
//}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	// Something is most likely wrong with the I2C bus
	// if we get to this point
	mrover::science.reboot_i2c();
}
