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

        fdcan_bus = FDCANBus{&hfdcan1};
        controller = BrushedController{CAN_ID, FusedReader{&htim4, &hi2c1}, HBridgeWriter{&htim15}, fdcan_bus};
    }

    void loop() {
        /* Commented out for pwm test
        // If the Receiver has messages waiting in its queue
        if (std::optional received = fdcan_bus.receive<InBoundMessage>()) {
            auto const& [header, message] = received.value();
            if (header.Identifier == CAN_ID)
                controller.receive(message);
        }

        controller.send();
    */

       Percent duty_cycle = make_unit<Percent>(-10.0);
       ThrottleCommand pwmtest;
       pwmtest.throttle = duty_cycle;
       controller.receive(pwmtest);
       controller.send();

    }

} // namespace mrover

void init() {
    mrover::init();
}

void loop() {
    mrover::loop();
}
