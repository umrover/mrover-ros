#include "controller.hpp"

#include <cstdint>

#include "hardware.hpp"
#include "main.h"
#include "messaging.hpp"
#include "reader.hpp"
#include "writer.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim4;

namespace mrover {

    using BrushedController = Controller<Radians, Percent, FusedReader, HBridgeWriter>;

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint32_t CAN_ID = 1;

    FDCANBus fdcan_bus;
    BrushedController controller;

    void init() {
        check(HAL_FDCAN_ActivateNotification(
                      &hfdcan1,
                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                      0) == HAL_OK,
              Error_Handler);
        check(HAL_FDCAN_Start(&hfdcan1) == HAL_OK, Error_Handler);

        fdcan_bus = FDCANBus{&hfdcan1};
        controller = BrushedController{CAN_ID, FusedReader{&htim4, &hi2c1}, HBridgeWriter{&htim15}, fdcan_bus};
    }

    void loop() {
        // If the Receiver has messages waiting in its queue
        if (std::optional received = fdcan_bus.receive<FdCanFrameIn>()) {
            auto const& [header, data] = received.value();
            if (header.Identifier == CAN_ID)
                controller.receive(data.message);
        }

        controller.send();
    }

} // namespace mrover

void init() {
    mrover::init();
}

void loop() {
    mrover::loop();
}
