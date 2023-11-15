#include "controller.hpp"

#include <cstdint>

#include <hardware.hpp>
#include <messaging.hpp>

#include "main.h"

// TODO: enable the watchdog and feed it in the htim6 update loop. make sure when the interrupt fires we disable PWN output. you will probably have to make the interrupt definition
// TODO: add another timer for absolute encoder? another solution is starting a transaction in the 10,000 Hz update loop if we are done with the previous transaction

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim4;  // Quadrature encoder #1
extern TIM_HandleTypeDef htim3;  // Quadrature encoder #2
extern TIM_HandleTypeDef htim6;  // 10,000 Hz Update timer
extern TIM_HandleTypeDef htim7;  // 100 Hz Send timer
extern TIM_HandleTypeDef htim15; // H-Bridge PWM
extern TIM_HandleTypeDef htim16; // Message watchdog timer

// extern WWDG_HandleTypeDef hwwdg;

namespace mrover {

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID = 0x1;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x0;

    FDCAN fdcan_bus;
    Controller controller;

    void init() {
        // Currently using polling based for CAN
         // check(HAL_FDCAN_ActivateNotification(
         //               &hfdcan1,
         //               FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
         //               0) == HAL_OK,
         //       Error_Handler);

        fdcan_bus = FDCAN{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        controller = Controller{
                &htim15,
                fdcan_bus,
                &htim16,
                &htim4,
                &hi2c1,
                {
                        LimitSwitch{Pin{LIMIT_0_0_GPIO_Port, LIMIT_0_0_Pin}},
                        LimitSwitch{Pin{LIMIT_0_1_GPIO_Port, LIMIT_0_1_Pin}},
                        LimitSwitch{Pin{LIMIT_0_2_GPIO_Port, LIMIT_0_2_Pin}},
                        LimitSwitch{Pin{LIMIT_0_3_GPIO_Port, LIMIT_0_3_Pin}},
                },
        };

        check(HAL_TIM_Base_Start_IT(&htim6) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(&htim7) == HAL_OK, Error_Handler);
    }

    void fdcan_received_callback() {
        std::optional received = fdcan_bus.receive<InBoundMessage>();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto messageId = std::bit_cast<FDCAN::MessageId>(header.Identifier);

        if (messageId.destination == DEVICE_ID) {
            controller.receive(message);
        }
    }

    void update_callback() {
        controller.update();
    }

    void send_callback() {
        controller.send();
    }

    void fdcan_watchdog_expired() {
        controller.receive_watchdog_expired();
    }

} // namespace mrover

extern "C" {

	void HAL_PostInit() {
		mrover::init();
	}

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
        if (htim == &htim6) {
            mrover::update_callback();
        } else if (htim == &htim7) {
            mrover::send_callback();
        } else if (htim == &htim16) {
            mrover::fdcan_watchdog_expired();
        }
    }

    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
            mrover::fdcan_received_callback();
        } else {
            // Mailbox is full OR we lost a frame
            Error_Handler();
        }
    }

    // TODO: error callback on FDCAN

    void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c) {}

    // void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    // }
    //
    // void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // }
    //
    // void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}

}
