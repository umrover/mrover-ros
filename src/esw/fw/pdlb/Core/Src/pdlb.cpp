#include <pdlb.hpp>
#include <cstdint>
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {

    // NOTE: Change this for the PDLB controller
    constexpr static std::uint8_t DEVICE_ID = 0x32;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCANBus fdcan_bus;
    PDLB pdlb;

    void init() {
        check(HAL_FDCAN_ActivateNotification(
                      &hfdcan1,
                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                      0) == HAL_OK,
              Error_Handler);

        fdcan_bus = FDCANBus{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        pdlb = PDLB{fdcan_bus};
    }

} // namespace mrover

void init() {
    mrover::init();
}

void update_and_send_current_temp() {
	mrover::update_and_send_current_temp();
}

void update_led() {
	mrover::update_led();
}

void receive_message() {
	mrover::receive_message();
}

