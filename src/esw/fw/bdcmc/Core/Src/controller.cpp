#include "controller.hpp"

#include <cstdint>

#include <hardware.hpp>
#include <messaging.hpp>

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;
#define ABSOLUTE_I2C &hi2c1

/**
 * For each timer, the update rate is determined by the .ioc file.
 *
 * Specifically the ARR value. You can use the following equation: ARR = (MCU Clock Speed) / (Update Rate) / (Prescaler + 1) - 1
 * For the STM32G4 we have a 140 MHz clock speed configured.
 *
 * You must also set auto reload to true so the interurpt gets called on a cycle.
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define QUADRATURE_TICK_TIMER_0 &htim3 // Special encoder timer which externally reads quadrature encoder ticks
#define QUADRATURE_TICK_TIMER_1 &htim4 // TODO update me!
#define ENCODER_ELAPSED_TIMER_0 &htim17 // Measures time since the last tick reading
#define ENCODER_ELAPSED_TIMER_1 &htim17 // TODO update me!
#define ABSOLUTE_ENCODER_TIMER &htim2   // Defines rate in which we send I2C request to absolute encoder
#define SEND_TIMER &htim7            // 100 Hz FDCAN repeating timer
#define PWM_TIMER_0 &htim15          // H-Bridge PWM
#define PWM_TIMER_1 &htim15          // TODO update me!
#define FDCAN_WATCHDOG_TIMER &htim16 // FDCAN watchdog timer that needs to be reset every time a message is received

namespace mrover {

    // NOTE: Change this for each motor controller
    constexpr static std::uint8_t DEVICE_ID_0 = 0x21; // currently set for joint_b

    // NOTE: Change this to 0x00 if there is no 2nd motor on the mcu
    constexpr static std::uint8_t DEVICE_ID_1 = 0x01; // currently set for nothing

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCAN<InBoundMessage> fdcan_bus;
    Controller controller0;
    std::optional<Controller> controller1;

    auto init() -> void {
        fdcan_bus = FDCAN<InBoundMessage>{0x00, DESTINATION_DEVICE_ID, &hfdcan1};

        controller0 = Controller{
                DEVICE_ID_0,
                PWM_TIMER_0,
                Pin{GPIOB, GPIO_PIN_15},
                fdcan_bus,
                FDCAN_WATCHDOG_TIMER,
                QUADRATURE_TICK_TIMER_0,
                ENCODER_ELAPSED_TIMER_0,
                ABSOLUTE_I2C,
                {
                        LimitSwitch{Pin{LIMIT_0_0_GPIO_Port, LIMIT_0_0_Pin}},
                        LimitSwitch{Pin{LIMIT_0_1_GPIO_Port, LIMIT_0_1_Pin}},
                        // LimitSwitch{Pin{LIMIT_0_2_GPIO_Port, LIMIT_0_2_Pin}},
                        // LimitSwitch{Pin{LIMIT_0_3_GPIO_Port, LIMIT_0_3_Pin}},
                },
        };

        if (DEVICE_ID_1 != 0x00) {
            std::array<LimitSwitch, 4> controller_1_limits = {
                LimitSwitch{Pin{LIMIT_1_0_GPIO_Port, LIMIT_1_0_Pin}},
                LimitSwitch{Pin{LIMIT_1_1_GPIO_Port, LIMIT_1_1_Pin}},
                // LimitSwitch{Pin{LIMIT_1_2_GPIO_Port, LIMIT_1_2_Pin}},
                // LimitSwitch{Pin{LIMIT_1_3_GPIO_Port, LIMIT_1_3_Pin}},
            };

            controller1.emplace(
                DEVICE_ID_1,
                PWM_TIMER_1,
                Pin{GPIOC, GPIO_PIN_6},
                fdcan_bus,
                FDCAN_WATCHDOG_TIMER,
                QUADRATURE_TICK_TIMER_1,
                ENCODER_ELAPSED_TIMER_1,
                ABSOLUTE_I2C,
                controller_1_limits);
        }

        // TODO: these should probably be in the controller / encoders themselves
        // Necessary for the timer interrupt to work
        // check(HAL_TIM_Base_Start_IT(UPDATE_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(SEND_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(ABSOLUTE_ENCODER_TIMER) == HAL_OK, Error_Handler);
    }

    auto fdcan_received_callback() -> void {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto messageId = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        if (messageId.destination == DEVICE_ID_0) {
            controller0.receive(message);
        } else if (messageId.destination == DEVICE_ID_1 && controller1) {
            controller1->receive(message);
        }
    }

    auto update_callback() -> void {
        controller0.update();
        if (controller1) controller1->update();
    }

    auto request_absolute_encoder_data_callback() -> void {
        controller0.request_absolute_encoder_data();
        if (controller1) controller1->request_absolute_encoder_data();
    }

    auto read_absolute_encoder_data_callback() -> void {
        controller0.read_absolute_encoder_data();
        if (controller1) controller1->read_absolute_encoder_data();
    }

    auto update_absolute_encoder_callback() -> void {
        controller0.update_absolute_encoder();
        if (controller1) controller1->update_absolute_encoder();
    }

    auto update_quadrature_encoder_0_callback() -> void {
        controller0.update_quadrature_encoder();
    }

    auto update_quadrature_encoder_1_callback() -> void {
        if (controller1) controller1->update_quadrature_encoder();
    }

    auto elapsed_timer_0_expired() -> void {
        controller0.elapsed_timer_expired();
    }

    auto elapsed_timer_1_expired() -> void {
        if (controller1) controller1->elapsed_timer_expired();
    }

    auto send_callback() -> void {
        controller0.send();
        if (controller1) controller1->send();
    }

    auto fdcan_watchdog_expired() -> void {
        controller0.receive_watchdog_expired();
        if (controller1) controller1->receive_watchdog_expired();
    }

} // namespace mrover

// TOOD: is this really necesssary?
extern "C" {

void HAL_PostInit() {
    mrover::init();
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
    if (htim == SEND_TIMER) {
        mrover::send_callback();
    } else if (htim == FDCAN_WATCHDOG_TIMER) {
        mrover::fdcan_watchdog_expired();
    } else if (htim == ENCODER_ELAPSED_TIMER_0) {
        mrover::elapsed_timer_0_expired();
    } else if (htim == ENCODER_ELAPSED_TIMER_1) {
        mrover::elapsed_timer_1_expired();
    } else if (htim == ABSOLUTE_ENCODER_TIMER) {
        mrover::request_absolute_encoder_data_callback();
    }
    // TODO: check for slow update timer and call on controller to send out i2c frame
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim == QUADRATURE_TICK_TIMER_0) {
        mrover::update_quadrature_encoder_0_callback();
    } else if (htim == QUADRATURE_TICK_TIMER_1) {
        mrover::update_quadrature_encoder_1_callback();
    }
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
