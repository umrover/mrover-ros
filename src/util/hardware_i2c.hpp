#pragma once

#include <any>
#include <cstdint>
#include <optional>

#include "hardware.hpp"

namespace mrover {

    constexpr static std::size_t I2C_MAX_FRAME_SIZE = 32;

    template<typename T>
    concept IsI2CSerializable = IsSerializable<T> && sizeof(T) <= I2C_MAX_FRAME_SIZE;

    class SMBus {
        constexpr static std::uint32_t I2C_TIMEOUT = 500, I2C_REBOOT_DELAY = 5;

    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c)
            : m_i2c{hi2c} {}

        void reboot() const {
            HAL_I2C_DeInit(m_i2c);
            HAL_Delay(I2C_REBOOT_DELAY);
            HAL_I2C_Init(m_i2c);
        }

        template<IsI2CSerializable TSend, IsI2CSerializable TReceive>
        auto blocking_transact(std::uint16_t address, TSend const& send) -> std::optional<TReceive> {
            if (HAL_I2C_Master_Transmit(m_i2c, address << 1, address_of<std::uint8_t>(send), sizeof(send), I2C_TIMEOUT) != HAL_OK) {
                // TODO(quintin): Do we want a different error handler here?
                return std::nullopt;
            }

            //reads from address sent above
            if (TReceive receive{}; HAL_I2C_Master_Receive(m_i2c, address << 1 | 1, address_of<std::uint8_t>(receive), sizeof(receive), I2C_TIMEOUT) != HAL_OK) {
                reboot();
                return std::nullopt;
            } else {
                return receive;
            }
        }

        template<IsI2CSerializable TSend>
        auto async_request(const std::uint16_t address, TSend const& send) -> void {
            // TODO: make sure actually sends to absolute encoder
            check(HAL_I2C_Master_Transmit_DMA(m_i2c, address << 1, address_of<std::uint8_t>(send), sizeof(send)) == HAL_OK, Error_Handler);

            // TODO: HAL recommends waiting until peripheral is done (same for Receive_DMA). Is this necessary?
            while (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY)
            {
            }
        }

        template<IsI2CSerializable TReceive>
        auto async_read(const std::uint16_t address) -> void {
            m_receive_buffer = TReceive{};
            check(HAL_I2C_Master_Receive_DMA(m_i2c, address << 1 | 1, address_of<std::uint8_t>(m_receive_buffer), sizeof(m_receive_buffer)) == HAL_OK, Error_Handler);
        }

        template<IsI2CSerializable TReceive>
        auto get_buffer() const -> std::optional<TReceive> {
            if (m_receive_buffer.type() != typeid(TReceive)) {
                return std::nullopt;
            }
            return std::any_cast<TReceive>(m_receive_buffer);
        }

    private:
        I2C_HandleTypeDef* m_i2c{};
        std::any m_receive_buffer{};
    };
} // namespace mrover
