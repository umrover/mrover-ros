#include <pdlb.hpp>
#include <cstdint>
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {

    // NOTE: Change this for the PDLB controller
    constexpr static std::uint8_t DEVICE_ID = 0x50;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCAN<InBoundPDLBMessage> fdcan_bus;
    PDLB pdlb;

    void init() {
        check(HAL_FDCAN_ActivateNotification(
                      &hfdcan1,
                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                      0) == HAL_OK,
              Error_Handler);

        AutonLed auton_led = AutonLed{
        	Pin{RED_LED_GPIO_Port, RED_LED_Pin},
			Pin{GREEN_LED_GPIO_Port, GREEN_LED_Pin},
			Pin{BLUE_LED_GPIO_Port, BLUE_LED_Pin}
        };
        std::shared_ptr<ADCSensor> adc_sensor_1 = std::make_shared<ADCSensor>(&hadc1, 10);
        std::shared_ptr<ADCSensor> adc_sensor_2 = std::make_shared<ADCSensor>(&hadc2, 2);

        std::array<CurrentSensor, 6> current_sensors =
		{
				CurrentSensor{adc_sensor_1, 0},
				CurrentSensor{adc_sensor_1, 1},
				CurrentSensor{adc_sensor_1, 2},
				CurrentSensor{adc_sensor_1, 3},
				CurrentSensor{adc_sensor_1, 4},
				CurrentSensor{adc_sensor_1, 5},

		};
        std::array<DiagTempSensor, 6> diag_temp_sensors =
        {
        		DiagTempSensor{adc_sensor_1, 6},
				DiagTempSensor{adc_sensor_1, 7},
				DiagTempSensor{adc_sensor_1, 8},
				DiagTempSensor{adc_sensor_1, 9},
				DiagTempSensor{adc_sensor_2, 0},
				DiagTempSensor{adc_sensor_2, 1},
        };

        fdcan_bus = FDCAN<InBoundPDLBMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        pdlb = PDLB{fdcan_bus, Pin{ARM_LASER_GPIO_Port, ARM_LASER_Pin},
        	auton_led, adc_sensor_1, adc_sensor_2,
			current_sensors, diag_temp_sensors};
    }

    void update_and_send_current_temp() {
    	pdlb.update_and_send_current_temp();
    }

    void blink_led_if_applicable() {
		pdlb.blink_led_if_applicable();
	}

    void receive_message() {
    	if (std::optional received = fdcan_bus.receive()) {
			auto const& [header, message] = received.value();
			auto messageId = std::bit_cast<FDCAN<InBoundPDLBMessage>::MessageId>(header.Identifier);
			if (messageId.destination == DEVICE_ID)
				pdlb.receive(message);
		}
	}

} // namespace mrover

void init() {
    mrover::init();
}

void update_and_send_current_temp() {
	mrover::update_and_send_current_temp();
}

void blink_led_if_applicable() {
	mrover::blink_led_if_applicable();
}

void receive_message() {
	mrover::receive_message();
}

