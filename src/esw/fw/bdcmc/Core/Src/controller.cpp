#include "controller.hpp"

#include "main.h"
#include "messaging.hpp"

#include <optional>

using namespace units::literals;

//extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;

constexpr size_t MAX_CONTROLLERS = 4;

std::array<std::optional<Controller>, MAX_CONTROLLERS> controllers;

template<size_t N>
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, uint32_t RxLocation, FDCAN_RxHeaderTypeDef* pRxHeader, std::array<std::byte, N>& RxData) {
    return HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, pRxHeader, reinterpret_cast<uint8_t*>(RxData.data()));
}

void init() {
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
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
        FdCanFrame frame{};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.bytes)) {
            Error_Handler();
        }
    }
}

void Controller::feed(ControlMessage const& message) {
    m_message = message;
}

void Controller::feed(OpenLoop const& message) {
}

void Controller::feed(VelocityControl const& message) {
}

void Controller::feed(PositionControl const& message) {
    auto yo = units::make_unit<units::angle::radian_t>(100);
    m_position_controller.calculate(yo, yo);
}

void Controller::step() {
    std::visit([&](auto const& arg) { feed(arg); }, m_message);
}
