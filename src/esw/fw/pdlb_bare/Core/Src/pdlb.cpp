#include <pdlb.hpp>
#include <cstdint>
#include "main.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim6; // for auton LED
#define AUTON_LED_TIMER &htim6

#define AUTON_LED_BLINK 5
uint8_t blink_count = 0;
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


        fdcan_bus = FDCAN<InBoundPDLBMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        pdlb = PDLB{fdcan_bus, Pin{ARM_LASER_GPIO_Port, ARM_LASER_Pin},
        	auton_led};
        check(HAL_TIM_Base_Start_IT(AUTON_LED_TIMER) == HAL_OK, Error_Handler);
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

void receive_message() {
	mrover::receive_message();
}

void blink_led_if_applicable() {
	mrover::blink_led_if_applicable();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == AUTON_LED_TIMER) {
		if (blink_count == AUTON_LED_BLINK) {
			blink_led_if_applicable();
			blink_count = 0;
		} else {
			++blink_count;
		}

	}
}
