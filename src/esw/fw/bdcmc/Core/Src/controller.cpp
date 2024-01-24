#include "controller.hpp"

#include <cstdint>

#include <hardware.hpp>
#include <messaging.hpp>

#include "main.h"

// Flag for testing
#define TESTING true

// TODO: enable the watchdog and feed it in the htim6 update loop. make sure when the interrupt fires we disable PWN output. you will probably have to make the interrupt definition
// TODO: add another timer for absolute encoder? another solution is starting a transaction in the 10,000 Hz update loop if we are done with the previous transaction

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;
#define ABSOLUTE_I2C &hi2c1

// extern WWDG_HandleTypeDef hwwdg;

/**
 * For each timer, the update rate is determined by the .ioc file.
 *
 * Specifically the ARR value. You can use the following equation: ARR = (MCU Clock Speed) / (Update Rate) / (Prescaler + 1) - 1
 * For the STM32G4 we have a 140 MHz clock speed configured.
 *
 * You must also set auto reload to true so the interurpt gets called on a cycle.
 */

extern TIM_HandleTypeDef htim4;  // Quadrature encoder #1
extern TIM_HandleTypeDef htim3;  // Quadrature encoder #2
extern TIM_HandleTypeDef htim6;  // 10,000 Hz Update timer
extern TIM_HandleTypeDef htim7;  // 100 Hz Send timer
extern TIM_HandleTypeDef htim15; // H-Bridge PWM
extern TIM_HandleTypeDef htim16; // Message watchdog timer
extern TIM_HandleTypeDef htim2; // Absolute encoder timer (currently at 20Hz)
#define QUADRATURE_TIMER_1 &htim4
#define QUADRATURE_TIMER_2 &htim3
#define UPDATE_TIMER &htim6
#define SEND_TIMER &htim7
#define PWM_TIMER &htim15
#define FDCAN_WATCHDOG_TIMER &htim16
#define ABSOLUTE_ENCODER_TIMER &htim2

namespace mrover {

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID = 0x1;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x0;

    FDCAN<InBoundMessage> fdcan_bus;
    Controller controller;

    void init() {
        fdcan_bus = FDCAN<InBoundMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        controller = Controller{
                PWM_TIMER,
				Pin{GPIOB, GPIO_PIN_15},
				Pin{GPIOA, GPIO_PIN_1},
                fdcan_bus,
                FDCAN_WATCHDOG_TIMER,
                QUADRATURE_TIMER_1,
                ABSOLUTE_I2C,
                {
                        LimitSwitch{Pin{LIMIT_0_0_GPIO_Port, LIMIT_0_0_Pin}},
                        LimitSwitch{Pin{LIMIT_0_1_GPIO_Port, LIMIT_0_1_Pin}},
                        LimitSwitch{Pin{LIMIT_0_2_GPIO_Port, LIMIT_0_2_Pin}},
                        LimitSwitch{Pin{LIMIT_0_3_GPIO_Port, LIMIT_0_3_Pin}},
                },
        };

        // Necessary for the timer interrupt to work
        check(HAL_TIM_Base_Start_IT(UPDATE_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(SEND_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(ABSOLUTE_ENCODER_TIMER) == HAL_OK, Error_Handler);
    }

    void fdcan_received_callback() {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto messageId = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        if (messageId.destination == DEVICE_ID) {
            controller.receive(message);
        }
    }

    void test_received_callback(InBoundMessage message) {
        controller.receive(message);
    }

    void update_callback() {
        controller.update();
    }

    void request_absolute_encoder_data_callback() {
        controller.request_absolute_encoder_data();
    }

    void read_absolute_encoder_data_callback() {
        controller.read_absolute_encoder_data();
    }

    void update_absolute_encoder_callback() {
        controller.update_absolute_encoder();
    }

    void send_callback() {
        controller.send();
    }

    void fdcan_watchdog_expired() {
        controller.receive_watchdog_expired();
    }

} // namespace mrover

// TOOD: is this really necesssary?
extern "C" {

void HAL_PostInit() {
    mrover::init();

//    #ifdef TESTING
////        const auto tests = mrover::get_test_msgs();
//        for (const auto& [test, delay] : tests) {
//            mrover::test_received_callback(test);
//            HAL_Delay(delay);
//        }
//    #endif
}

/**
* These are interrupt handlers. They are called by the HAL.
*
* These are set up in the .ioc file.
* They have to be enabled in the NVIC settings.
* It is important th
*/

/**
 * \note Timers have to be started with "HAL_TIM_Base_Start_IT" for this interrupt to work for them.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == UPDATE_TIMER) {
        mrover::update_callback();
    } else if (htim == SEND_TIMER) {
        mrover::send_callback();
    } else if (htim == FDCAN_WATCHDOG_TIMER) {
        mrover::fdcan_watchdog_expired();
    } else if (htim == ABSOLUTE_ENCODER_TIMER) {
        mrover::request_absolute_encoder_data_callback();
    }
    // TODO: check for slow update timer and call on controller to send out i2c frame
}

/**
 * \note FDCAN1 has to be configured with "HAL_FDCAN_ActivateNotification" and started with "HAL_FDCAN_Start" for this interrupt to work.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        mrover::fdcan_received_callback();
    } else {
        // Mailbox is full OR we lost a frame
        Error_Handler();
    }
}

// TODO: implement

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {}

    // TODO DMA receive callback, this should eventually call a function on the controller with most up to date info

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::update_absolute_encoder_callback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::read_absolute_encoder_data_callback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}
}
