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
    constexpr static std::uint8_t DEVICE_ID = 0x1;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x0;

    FDCANBus fdcan_bus;
    BrushedController controller;

    void init() {
        check(HAL_FDCAN_ActivateNotification(
                      &hfdcan1,
                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                      0) == HAL_OK,
              Error_Handler);

        fdcan_bus = FDCANBus{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        std::array<LimitSwitch, 4> limit_switches = {
        		LimitSwitch{Pin{LIMIT_0_0_GPIO_Port, LIMIT_0_0_Pin}},
				LimitSwitch{Pin{LIMIT_0_1_GPIO_Port, LIMIT_0_1_Pin}},
				LimitSwitch{Pin{LIMIT_0_2_GPIO_Port, LIMIT_0_2_Pin}},
				LimitSwitch{Pin{LIMIT_0_3_GPIO_Port, LIMIT_0_3_Pin}}
        };
        controller = BrushedController{FusedReader{&htim4, &hi2c1}, HBridgeWriter{&htim15}, limit_switches, fdcan_bus};
    }

    void loop() {
        // If the Receiver has messages waiting in its queue
        if (std::optional received = fdcan_bus.receive<InBoundMessage>()) {
            auto const& [header, message] = received.value();
            auto messageId = std::bit_cast<FDCANBus::MessageId>(header.Identifier);
            if (messageId.destination == DEVICE_ID)
                controller.receive(message);
        }

        HAL_Delay(100); // TODO: remove after debugging

        controller.update_and_send();
    }

} // namespace mrover

void init() {
    mrover::init();
}

void loop() {
    mrover::loop();
}
