#include "controller.hpp"

#include <span>

#include "main.h"
#include "messaging.hpp"
#include "writer.hpp"
#include "reader.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

constexpr uint32_t CAN_ID = 1;

namespace mrover {

    // Motor Controller Definitions Here
    Controller<Radians, Volts, EncoderReader, BrushedMotorWriter> controller(CAN_ID);

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
    mrover::controller.update();

    // If the Receiver has messages waiting in its queue
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef header;
        mrover::FdCanFrame frame{};

        check(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.bytes) == HAL_OK, Error_Handler);
        if (header.Identifier == CAN_ID) {
            mrover::controller.process(frame.message);
        }
    }

    // TODO: send out Controller data as CAN message
    // Use getters to get info from encoder reader, have the bundling into CAN in controller
    //  then controller sends the array as a CAN message
    /* CAN FRAME FORMAT 
     * 4 BYTES: Position (0-3). In RADIANS
     * 4 BYTES: Velocity (4-7). In RADIANS/SECOND
     * 1 BYTES: Is Configured, Is Calibrated, Errors (8)
     * 1 BYTES: Limit a/b/c/d is hit. (9)
     */
    mrover::controller.transmit_motor_data();
}