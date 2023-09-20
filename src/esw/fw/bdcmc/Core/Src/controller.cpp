#include "controller.hpp"

#include <span>

#include "main.h"
#include "messaging.hpp"
#include "writer.hpp"
#include "reader.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {

    // Motor Controller Definitions Here
    Controller<Radians, Volts, EncoderReader, BrushedMotorWriter> controller;

} // namespace mrover

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, uint32_t RxLocation, FDCAN_RxHeaderTypeDef* pRxHeader, std::span<std::byte, mrover::FRAME_SIZE> RxData) {
    return HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, pRxHeader, reinterpret_cast<uint8_t*>(RxData.data()));
}

void init() {
    check(HAL_FDCAN_ActivateNotification(
                  &hfdcan1,
                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                  0) == HAL_OK,
          Error_Handler);
    check(HAL_FDCAN_Start(&hfdcan1) == HAL_OK, Error_Handler);
}

void loop() {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef header;
        mrover::FdCanFrame frame{};
        check(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.bytes) == 0, Error_Handler);
        mrover::controller.update(frame.message);
    }
}
