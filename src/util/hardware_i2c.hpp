#pragma once

#include "main.h"
#include "messaging.hpp"

#include "hardware.hpp"
#include <bit>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>

namespace mrover {

    constexpr static std::size_t I2C_MAX_FRAME_SIZE = 32;

    template<typename T>
    concept IsI2CSerializable = IsSerializable<T> && sizeof(T) <= mrover::I2C_MAX_FRAME_SIZE;

    class SMBus {
        constexpr static std::uint32_t I2C_TIMEOUT = 500, I2C_REBOOT_DELAY = 5;

        void reboot() {
            HAL_I2C_DeInit(m_i2c);
            HAL_Delay(I2C_REBOOT_DELAY);
            HAL_I2C_Init(m_i2c);
        }

    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c) : m_i2c{hi2c} {
        }

        template<IsI2CSerializable TSend, IsI2CSerializable TReceive>
        auto transact(std::uint16_t address, TSend const& send) -> std::optional<TReceive> {
            if (HAL_I2C_Master_Transmit(m_i2c, address << 1, address_of<std::uint8_t>(send), sizeof(send), I2C_TIMEOUT) != HAL_OK) {
                // TODO(quintin): Do we want a different error handler here?
                return std::nullopt;
            }

            //reads from address sent above
            if (TReceive receive{}; HAL_I2C_Master_Receive(m_i2c, (address << 1) | 1, address_of<std::uint8_t>(receive), sizeof(receive), I2C_TIMEOUT) == HAL_OK) {
                return receive;
            } else {
                reboot();
                return std::nullopt;
            }
        }

    private:
        I2C_HandleTypeDef* m_i2c{};
    };
} // namespace mrover
