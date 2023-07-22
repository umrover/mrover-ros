#include "controller.hpp"

#include "main.h"
#include "messaging.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {

    class EncoderReader {
        TIM_HandleTypeDef* m_relative_encoder_timer = nullptr;
        I2C_HandleTypeDef* m_absolute_encoder_i2c = nullptr;

    public:
        EncoderReader() = default;

        EncoderReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c)
            : m_relative_encoder_timer{relative_encoder_timer}, m_absolute_encoder_i2c{absolute_encoder_i2c} {

            // TODO: initialize with HAL
            // TODO: store state to center the relative encoder readings based on absolute
        }

        [[nodiscard]] Radians read_input() {

            // TODO: read relative encoder using timer

            return {};
        }
    };

    struct BrushedMotorWriter {
        void write_output(Volts output) {
        }
    };

    Controller<Radians, Volts, EncoderReader, BrushedMotorWriter> controller;

} // namespace mrover

template<size_t N>
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, uint32_t RxLocation, FDCAN_RxHeaderTypeDef* pRxHeader, std::array<std::byte, N>& RxData) {
    return HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, pRxHeader, reinterpret_cast<uint8_t*>(RxData.data()));
}

void init() {
    if (HAL_FDCAN_ActivateNotification(
                &hfdcan1,
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_LOGGING_OVERFLOW,
                0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

void loop() {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef header;
        mrover::FdCanFrame frame{};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.bytes)) {
            Error_Handler();
        }
        mrover::controller.update(frame.message);
    }
}
