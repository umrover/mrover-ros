#pragma once

#include "main.h"
#include "messaging.hpp"

#include <bit>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>

namespace mrover {

    constexpr static std::size_t I2C_MAX_FRAME_SIZE = 32, FDCAN_MAX_FRAME_SIZE = 64;

    template<typename T>
    concept IsSerializable = std::is_trivially_copyable_v<T>;

    template<typename T>
    concept IsI2CSerializable = IsSerializable<T> && sizeof(T) <= mrover::I2C_MAX_FRAME_SIZE;

    template<typename T>
    concept IsFdcanSerializable = IsSerializable<T> && sizeof(T) <= mrover::FDCAN_MAX_FRAME_SIZE;

    static inline auto check(bool cond, std::invocable auto handler) -> void {
        if (cond) return;

        handler();
    }

    template<typename T>
    static inline auto address_of(auto const& object) -> T* {
        return std::bit_cast<T*>(std::addressof(object));
    }

    class Pin {
    public:
        Pin() = default;

        Pin(GPIO_TypeDef* port, std::uint16_t pin) : m_port{port}, m_pin{pin} {}

        [[nodiscard]] inline GPIO_PinState read() {
            return HAL_GPIO_ReadPin(m_port, m_pin);
        }

        inline void write(GPIO_PinState val) {
            HAL_GPIO_WritePin(m_port, m_pin, val);
        }

    private:
        GPIO_TypeDef* m_port{};
        std::uint16_t m_pin{};
    };

    class LimitSwitch {
    public:
        LimitSwitch() = default;

        explicit LimitSwitch(Pin const& pin) : m_pin{pin} {}

        void update_limit_switch() {
            // This suggests active low
            if (m_enabled) {
                m_is_pressed = m_active_high == m_pin.read();
            } else {
                m_is_pressed = 0;
            }
        }

        [[nodiscard]] bool pressed() const {
            return m_is_pressed;
        }

        void enable() {
            m_enabled = true;
        }

    private:
        Pin m_pin;
        std::uint8_t m_enabled{};
        std::uint8_t m_is_pressed{};
        std::uint8_t m_valid{};
        std::uint8_t m_active_high{};
        std::int32_t m_associated_count{};
    };

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

    class FDCANBus {
        //        constexpr static std::uint8_t NO_DATA = 0x50;

    public:
        struct MessageId {
            std::uint8_t destination{};
            std::uint8_t source : 7 {};
            bool replyRequired : 1 {};
            [[maybe_unsued]] std::uint16_t _ignored{};
        };
        static_assert(sizeof(MessageId) == 4);

        FDCANBus() = default;

        explicit FDCANBus(std::uint8_t source, std::uint8_t destination, FDCAN_HandleTypeDef* fdcan)
            : m_fdcan{fdcan}, m_source{source}, m_destination{destination} {

            check(HAL_FDCAN_Start(m_fdcan) == HAL_OK, Error_Handler);
        }

        template<IsFdcanSerializable TReceive>
        [[nodiscard]] auto receive() -> std::optional<std::pair<FDCAN_RxHeaderTypeDef, TReceive>> {
            if (HAL_FDCAN_GetRxFifoFillLevel(m_fdcan, FDCAN_RX_FIFO0)) {
                FDCAN_RxHeaderTypeDef header{};
                TReceive receive{};
                check(HAL_FDCAN_GetRxMessage(m_fdcan, FDCAN_RX_FIFO0, &header, address_of<std::uint8_t>(receive)) == HAL_OK, Error_Handler);
                return std::make_pair(header, receive);
            } else {
                return std::nullopt;
            }
        }

        consteval static auto nearest_fitting_can_fd_frame_size(std::size_t size) -> std::uint32_t {
            if (size <= 0) return FDCAN_DLC_BYTES_0;
            if (size <= 1) return FDCAN_DLC_BYTES_1;
            if (size <= 2) return FDCAN_DLC_BYTES_2;
            if (size <= 3) return FDCAN_DLC_BYTES_3;
            if (size <= 4) return FDCAN_DLC_BYTES_4;
            if (size <= 5) return FDCAN_DLC_BYTES_5;
            if (size <= 6) return FDCAN_DLC_BYTES_6;
            if (size <= 7) return FDCAN_DLC_BYTES_7;
            if (size <= 8) return FDCAN_DLC_BYTES_8;
            if (size <= 12) return FDCAN_DLC_BYTES_12;
            if (size <= 16) return FDCAN_DLC_BYTES_16;
            if (size <= 20) return FDCAN_DLC_BYTES_20;
            if (size <= 24) return FDCAN_DLC_BYTES_24;
            if (size <= 32) return FDCAN_DLC_BYTES_32;
            if (size <= 48) return FDCAN_DLC_BYTES_48;
            if (size <= 64) return FDCAN_DLC_BYTES_64;
            // exception handling disabled, use '-fexceptions' to enable
            // project -> Properties -> C/C++ Build -> Settings ->
            // Tool Settings -> MCU G++ Compiler -> Miscellaneous
            // Add the "-fexceptions" flag there.
            throw std::runtime_error("Too large!");
        }

        auto broadcast(IsFdcanSerializable auto const& send) -> void {
            MessageId messageId{
                    .destination = m_destination,
                    .source = m_source,
            };
            FDCAN_TxHeaderTypeDef header{
                    .Identifier = std::bit_cast<std::uint32_t>(messageId),
                    .IdType = FDCAN_EXTENDED_ID,
                    .TxFrameType = FDCAN_DATA_FRAME,
                    .DataLength = nearest_fitting_can_fd_frame_size(sizeof(send)),
                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                    .BitRateSwitch = FDCAN_BRS_ON,
                    .FDFormat = FDCAN_FD_CAN,
                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            };
            if (HAL_FDCAN_GetTxFifoFreeLevel(m_fdcan)) {
                check(HAL_FDCAN_AddMessageToTxFifoQ(m_fdcan, &header, address_of<std::uint8_t>(send)) == HAL_OK, Error_Handler);
            } else {
                // TODO: somehow convey that this is an error
                HAL_FDCAN_AbortTxRequest(m_fdcan, FDCAN_TX_BUFFER0);
            }
        }

    private:
        FDCAN_HandleTypeDef* m_fdcan{};
        std::uint8_t m_source{}, m_destination{};
    };

} // namespace mrover
