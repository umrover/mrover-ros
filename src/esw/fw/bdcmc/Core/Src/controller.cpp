#include "controller.hpp"

#include <span>

#include "main.h"
#include "messaging.hpp"
#include "reader.hpp"
#include "writer.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;

// NOTE: Change This For Each Motor Controller
constexpr std::uint32_t CAN_ID = 1;

using controller_t = mrover::Controller<mrover::Radians, mrover::Dimensionless, mrover::EncoderReader, mrover::BrushedMotorWriter>;

namespace mrover {
    std::optional<controller_t> controller;
} // namespace mrover

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, std::uint32_t RxLocation, FDCAN_RxHeaderTypeDef* pRxHeader, std::span<std::byte, mrover::FRAME_SIZE> RxData) {
    return HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, pRxHeader, reinterpret_cast<uint8_t*>(RxData.data()));
}

void init() {
    check(HAL_FDCAN_ActivateNotification(
                  &hfdcan1,
                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                  0) == HAL_OK,
          Error_Handler);
    check(HAL_FDCAN_Start(&hfdcan1) == HAL_OK, Error_Handler);

    mrover::controller.emplace(CAN_ID, mrover::EncoderReader(&htim3, &hi2c1), mrover::BrushedMotorWriter(&htim15));
}

void loop() {
    // If the Receiver has messages waiting in its queue
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef header;
        mrover::FdCanFrameIn frame{};

        check(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.bytes) == HAL_OK, Error_Handler);
        if (header.Identifier == CAN_ID) {
            mrover::controller->process(frame.message);
        }
    }

    mrover::controller->update();

    mrover::controller->transmit();
}
