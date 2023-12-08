#pragma once

#include <bit>
#include <bitset>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>

#include <units/units.hpp>

#include "main.h"

namespace mrover {

    constexpr static std::size_t FDCAN_MAX_FRAME_SIZE = 64;

    template<typename T>
    concept IsSerializable = std::is_trivially_copyable_v<T>;

    template<typename T>
    concept IsFdcanSerializable = IsSerializable<T> && sizeof(T) <= FDCAN_MAX_FRAME_SIZE;

    static auto check(bool cond, std::invocable auto handler) -> void {
        if (cond) return;

        handler();
    }

    template<typename T>
    static auto address_of(auto const& object) -> T* {
        return std::bit_cast<T*>(std::addressof(object));
    }

    class Pin {
    public:
        Pin() = default;

        Pin(GPIO_TypeDef* port, std::uint16_t pin)
            : m_port{port}, m_pin{pin} {}

        [[nodiscard]] auto read() const -> GPIO_PinState {
            return HAL_GPIO_ReadPin(m_port, m_pin);
        }

        inline auto write(GPIO_PinState val) const -> void {
            HAL_GPIO_WritePin(m_port, m_pin, val);
        }

    private:
        GPIO_TypeDef* m_port{};
        std::uint16_t m_pin{};
    };

    class LimitSwitch {
    public:
        LimitSwitch() = default;

        explicit LimitSwitch(Pin const& pin)
            : m_pin{pin} {}

        auto initialize(bool enabled, bool active_high, bool used_for_readjustment, bool limits_forward, Radians associated_position) -> void {
            m_valid = true;
            m_enabled = enabled;
            m_is_pressed = false;
            m_active_high = active_high;
            m_used_for_readjustment = used_for_readjustment;
            m_limits_forward = limits_forward;
            m_associated_position = associated_position;
        }

        auto update_limit_switch() -> void {
            // This suggests active low
            if (m_enabled) {
                m_is_pressed = m_active_high == m_pin.read();
            } else {
                m_is_pressed = false;
            }
        }

        [[nodiscard]] auto pressed() const -> bool {
            return m_is_pressed;
        }

        [[nodiscard]] auto limit_forward() const -> bool {
            return m_valid && m_enabled && m_is_pressed && m_limits_forward;
        }

        [[nodiscard]] auto limit_backward() const -> bool {
            return m_valid && m_enabled && m_is_pressed && !m_limits_forward;
        }

        [[nodiscard]] auto get_readjustment_position() const -> std::optional<Radians> {
            // Returns std::null_opt if the value should not be readjusted
            if (m_valid && m_enabled && m_used_for_readjustment && m_is_pressed) {
                return m_associated_position;
            } else {
                return std::nullopt;
            }
        }

        void enable() {
            m_enabled = true;
        }

        void disable() {
            m_enabled = false;
            m_is_pressed = false;
        }

    private:
        Pin m_pin;
        bool m_valid{};
        bool m_enabled{};
        bool m_is_pressed{};
        bool m_active_high{};
        bool m_used_for_readjustment{};
        bool m_limits_forward{};
        Radians m_associated_position{};
    };

    template<IsFdcanSerializable TReceive>
    class FDCAN {
        //        constexpr static std::uint8_t NO_DATA = 0x50;

    public:
        struct MessageId {
            std::uint8_t destination{};
            std::uint8_t source : 7 {};
            bool replyRequired : 1 {};
            [[maybe_unused]] std::uint16_t _ignored{};
        };

        static_assert(sizeof(MessageId) == 4);

        FDCAN() = default;

        explicit FDCAN(std::uint8_t source, std::uint8_t destination, FDCAN_HandleTypeDef* fdcan)
            : m_fdcan{fdcan}, m_source{source}, m_destination{destination} {

            configure_filter();

            check(HAL_FDCAN_ActivateNotification(m_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK, Error_Handler);
            check(HAL_FDCAN_Start(m_fdcan) == HAL_OK, Error_Handler);
        }

        /**
         * By default CAN is a broadcast protocol, meaning that all devices on the bus receive all messages.
         * We have limited queue space, so we want to filter out messages that are not intended for us.
         */
        auto configure_filter() const -> void {
            std::bitset<16> source_bits{m_source};
            std::bitset<16> destinationBits{m_destination};
            std::bitset<16> filter = source_bits << 8 | destinationBits;

            std::bitset<15> mask; // 15 lowest bits set for source and destination, 16th (reply bit) and above should be ignored
            mask.set();

            FDCAN_FilterTypeDef filter_config{
                    .IdType = FDCAN_EXTENDED_ID,
                    .FilterIndex = 0,
                    .FilterType = FDCAN_FILTER_MASK, // Classic filter: FilterID1 = filter, FilterID2 = mask
                    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
                    .FilterID1 = filter.to_ulong(), // Choose which bits to examine from the incoming ID
                    .FilterID2 = mask.to_ulong(),   // Ensure that bits that survive the filter match the mask
            };
            check(HAL_FDCAN_ConfigFilter(m_fdcan, &filter_config) == HAL_OK, Error_Handler);
        }

        /**
         * \brief           Attempt to pop a message from the receive queue
         * \tparam TReceive The type of the message to receive
         * \return          Header and message, or none if queue is empty
         */
        [[nodiscard]] auto receive() -> std::optional<std::pair<FDCAN_RxHeaderTypeDef, TReceive>> {
            if (HAL_FDCAN_GetRxFifoFillLevel(m_fdcan, FDCAN_RX_FIFO0) == 0)
                return std::nullopt;

            FDCAN_RxHeaderTypeDef header{};
            TReceive receive{};
            check(HAL_FDCAN_GetRxMessage(m_fdcan, FDCAN_RX_FIFO0, &header, address_of<std::uint8_t>(receive)) == HAL_OK, Error_Handler);
            return std::make_pair(header, receive);
        }

        /**
         * \brief Needed since only certain frame sizes less than or equal to 64 bytes are allowed
         */
        [[nodiscard]] consteval static auto nearest_fitting_can_fd_frame_size(std::size_t size) -> std::uint32_t {
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
            // Need to cause a compile error as this size is not feasible to put in a single FDCAN frame
            // Trying to call a non-consteval function will do this
            Error_Handler();
            return {};
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
            // Sending a message to the bus gets added to a queue
            // Messages in the queue are removed when they are sent out
            // They are sent out when the bus is free AND there is at least one other device on the bus
            // This means you cannot test with just one device on the bus
            if (HAL_FDCAN_GetTxFifoFreeLevel(m_fdcan)) {
                // Free space in the mailbox
                check(HAL_FDCAN_AddMessageToTxFifoQ(m_fdcan, &header, address_of<std::uint8_t>(send)) == HAL_OK, Error_Handler);
            } else {
                // Abort oldest message in the mailbox to make room for the new message
                // TODO: somehow convey that this is an error
                HAL_FDCAN_AbortTxRequest(m_fdcan, FDCAN_TX_BUFFER0);
            }
        }

    private:
        FDCAN_HandleTypeDef* m_fdcan{};
        std::uint8_t m_source{}, m_destination{};
    };

} // namespace mrover