#include "controller.hpp"

#include "main.h"
#include "messaging.hpp"

#include <numbers>
#include <span>

extern FDCAN_HandleTypeDef hfdcan1;

constexpr uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;

static inline void check(bool cond, std::invocable auto handler) {
    if (!cond) {
        handler();
    }
}

namespace mrover {

    class EncoderReader {
    public:
        EncoderReader() = default;

        EncoderReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c)
            : m_relative_encoder_timer{relative_encoder_timer}, m_absolute_encoder_i2c{absolute_encoder_i2c} {
            // Initialize the TIM and I2C encoders
            check(HAL_TIM_Encoder_Init(m_relative_encoder_timer, nullptr /* TODO: replace with config */) == HAL_OK, Error_Handler);
            check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);
            check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
            // Store state to center the relative encoder readings based on absolute
            refresh_absolute();
        }

        void refresh_absolute() {
            uint32_t count_from_absolute_encoder = 0; // TODO: get absolute count here
            // Get reading from absolute encoder
            rotation = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder;
            // Calculate how much ahead/behind the absolute encoder's reading is with respect to the relative count
            absolute_relative_diff = rotation - RADIANS_PER_COUNT_RELATIVE * m_relative_encoder_timer->Instance->CNT;
        }

        void update_count() {
            rotation = RADIANS_PER_COUNT_RELATIVE * m_relative_encoder_timer->Instance->CNT + absolute_relative_diff;
        }

        [[nodiscard]] Radians read_input() const {
            return rotation;
        }

    private:
        TIM_HandleTypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians rotation{};
        Radians absolute_relative_diff{};
    };

    struct BrushedMotorWriter {
        void write_output(Volts output) {
            // TODO implement
        }
    };

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
