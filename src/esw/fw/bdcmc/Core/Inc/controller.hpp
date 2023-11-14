#pragma once

#include <array>
#include <concepts>
#include <optional>
#include <type_traits>
#include <variant>

#include <hardware.hpp>
#include <messaging.hpp>
#include <pidf.hpp>
#include <units/units.hpp>

#include "encoders.hpp"
#include "hbridge.hpp"

namespace mrover {

    class Controller {
        using MotorDriver = HBridge;

        using Encoder = std::variant<
                std::monostate, FusedReader, QuadratureEncoderReader, AbsoluteEncoderReader>;

        struct PositionMode {
            PIDF<Radians, Percent> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<Radians, inverse<Seconds>>, Percent> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        /* Hardware */
        FDCAN m_fdcan;
        MotorDriver m_motor_driver;
        Encoder m_encoder;
        TIM_HandleTypeDef* m_watchdog_timer{};
        bool m_watchdog_enabled{};
        TIM_HandleTypeDef* m_quadrature_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        std::array<LimitSwitch, 4> m_limit_switches;

        /* Internal State */
        Mode m_mode;
        // General State
        Percent m_desired_output; // "Desired" since it may be overriden if we are at a limit switch
        std::optional<Radians> m_position;
        std::optional<RadiansPerSecond> m_velocity;
        BDCMCErrorInfo m_error = BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED;
        // Configuration State
        struct StateAfterConfig {
            Dimensionless gear_ratio;
            Radians max_forward_pos;
            Radians max_backward_pos;
        };
        std::optional<StateAfterConfig> m_state_after_config;
        // Calibration State
        struct StateAfterCalib {
            // Ex: If the encoder reads in 6 Radians and offset is 4 Radians,
            // Then my actual current position should be 2 Radians.
            Radians offset_position;
        };
        std::optional<StateAfterCalib> m_state_after_calib;
        // Messaging
        InBoundMessage m_inbound = IdleCommand{};
        OutBoundMessage m_outbound = ControllerDataState{.config_calib_error_data = {.error = m_error}};

        /**
         * \brief Updates \link m_position and \link m_velocity based on the hardware
         */
        auto update_encoder() -> void {
            std::optional<EncoderReading> reading;

            // Only read the encoder if we are configured
            if (m_state_after_config) {
                reading = std::visit(
                        overloaded{
                                [&](std::monostate) -> std::optional<EncoderReading> { return std::nullopt; },
                                [&](auto& reader) -> std::optional<EncoderReading> { return reader.read(); },
                        },
                        m_encoder);
            }

            if (reading) {
                auto const& [position, velocity] = reading.value();
                if (std::holds_alternative<QuadratureEncoderReader>(m_encoder)) {
                    // For relative encoder update position only if we are calibrated (otherwise we don't know our absolute position)
                    if (m_state_after_calib)
                        m_position = position - m_state_after_calib->offset_position;
                } else {
                    m_position = position;
                }
                m_velocity = velocity;
            } else {
                m_position = std::nullopt;
                m_velocity = std::nullopt;
            }
        }

        auto update_limit_switches() -> void {
            for (LimitSwitch& limit_switch: m_limit_switches) {
                limit_switch.update_limit_switch();
                // Each limit switch may have a position associated with it
                // If we reach there update our offset position since we know exactly where we are
                if (limit_switch.pressed() && m_position && m_state_after_calib) {
                    if (std::optional<Radians> readjustment_position = limit_switch.get_readjustment_position()) {
                        m_state_after_calib->offset_position = m_position.value() - readjustment_position.value();
                    }
                }
            }
        }

        auto drive_motor() -> void {
            std::optional<Percent> output;

            if (m_state_after_config) {
                bool limit_forward = m_desired_output > 0_percent && std::ranges::any_of(m_limit_switches, [](LimitSwitch const& limit_switch) { return limit_switch.limit_forward(); });
                bool limit_backward = m_desired_output < 0_percent && std::ranges::any_of(m_limit_switches, [](LimitSwitch const& limit_switch) { return limit_switch.limit_backward(); });
                if (limit_forward || limit_backward) {
                    m_error = BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS;
                } else {
                    output = m_desired_output;
                }
            }

            m_motor_driver.write(output.value_or(0_percent));
        }

        auto process_command(AdjustCommand const& message) -> void {
            update_encoder();

            if (m_position && m_state_after_config) {
                m_state_after_calib = StateAfterCalib{
                        .offset_position = m_position.value() - message.position,
                };
            }
        }

        auto process_command(ConfigCommand const& message) -> void {
            // Initialize values
            StateAfterConfig config{.gear_ratio = message.gear_ratio};

            if (message.quad_abs_enc_info.quad_present && message.quad_abs_enc_info.abs_present) {
                Ratio quad_multiplier = (message.quad_abs_enc_info.quad_is_forward_polarity ? 1 : -1) * message.quad_enc_out_ratio;

                Ratio abs_multiplier = (message.quad_abs_enc_info.abs_is_forward_polarity ? 1 : -1) * message.abs_enc_out_ratio;

                m_encoder = FusedReader{m_quadrature_encoder_timer, m_absolute_encoder_i2c, quad_multiplier, abs_multiplier};

            } else if (message.quad_abs_enc_info.quad_present) {
                // TODO(quintin): Why TF does this crash without .get() ?
                Ratio multiplier = (message.quad_abs_enc_info.quad_is_forward_polarity ? 1.0f : -1.0f) * message.quad_enc_out_ratio.get();

                m_encoder = QuadratureEncoderReader{m_quadrature_encoder_timer, multiplier};
            } else if (message.quad_abs_enc_info.abs_present) {
                Ratio multiplier = (message.quad_abs_enc_info.abs_is_forward_polarity ? 1 : -1) * message.abs_enc_out_ratio;

                // A1 and A2 are grounded
                m_encoder = AbsoluteEncoderReader{SMBus{m_absolute_encoder_i2c}, 0, 0, multiplier};
            }

            m_motor_driver.change_max_pwm(message.max_pwm);

            config.max_forward_pos = message.max_forward_pos;
            config.max_backward_pos = message.max_backward_pos;

            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i)) {
                    bool enabled = GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i);
                    bool active_high = GET_BIT_AT_INDEX(message.limit_switch_info.active_high, i);
                    bool used_for_readjustment = GET_BIT_AT_INDEX(message.limit_switch_info.use_for_readjustment, i);
                    bool limits_forward = GET_BIT_AT_INDEX(message.limit_switch_info.limits_forward, i);
                    auto associated_position = Radians{message.limit_switch_info.limit_readj_pos[i]};

                    m_limit_switches[i].initialize(enabled, active_high, used_for_readjustment, limits_forward, associated_position);
                }
            }

            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i) && GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i)) {
                    m_limit_switches[i].enable();
                }
            }

            m_state_after_config = config;

            update_encoder();
        }

        auto process_command(IdleCommand const&) -> void {
            // TODO - what is the expected behavior? just afk?
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = 0_percent;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(ThrottleCommand const& message) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = message.throttle;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(VelocityCommand const& message, VelocityMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            update_encoder();

            if (!m_velocity) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            RadiansPerSecond target = message.velocity;
            RadiansPerSecond input = m_velocity.value();
            m_desired_output = mode.pidf.calculate(input, target);
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(PositionCommand const& message, PositionMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }
            if (!m_state_after_calib) {
                m_error = BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED;
                return;
            }

            update_encoder();

            if (!m_position) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            Radians target = message.position;
            Radians input = m_position.value();
            m_desired_output = mode.pidf.calculate(input, target);
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(EnableLimitSwitchesCommand const&) -> void {
            // We are allowed to just enable all limit switches.
            // The valid bit is kept track of separately.
            for (int i = 0; i < 4; ++i) {
                m_limit_switches[i].enable();
            }
        }

        struct detail {
            template<typename Command, typename V>
            struct command_to_mode;

            template<typename Command, typename ModeHead, typename... Modes>
            struct command_to_mode<Command, std::variant<ModeHead, Modes...>> { // Linear search to find corresponding mode
                using type = std::conditional_t<requires(Controller controller, Command command, ModeHead mode) { controller.process_command(command, mode); },
                                                ModeHead,
                                                typename command_to_mode<Command, std::variant<Modes...>>::type>; // Recursive call
            };

            template<typename Command> // Base case
            struct command_to_mode<Command, std::variant<>> {
                using type = std::monostate;
            };
        };

        template<typename Command, typename T>
        using command_to_mode_t = typename detail::template command_to_mode<Command, T>::type;

    public:
        Controller() = default;

        Controller(TIM_HandleTypeDef* hbridge_output, FDCAN const& fdcan, TIM_HandleTypeDef* watchdog_timer, TIM_HandleTypeDef* quadrature_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c, std::array<LimitSwitch, 4> const& limit_switches)
            : m_motor_driver{HBridge(hbridge_output)},
              m_fdcan{fdcan},
              m_watchdog_timer{watchdog_timer},
              m_quadrature_encoder_timer{quadrature_encoder_timer},
              m_absolute_encoder_i2c{absolute_encoder_i2c},
              m_limit_switches{limit_switches} {}

        template<typename Command>
        auto process_command(Command const& command) -> void {
            // Find the "process_command" function that has the right type for the command
            using ModeForCommand = command_to_mode_t<Command, Mode>;

            // If the current mode is not the mode that "process_command" expects, change the mode, providing a new blank mode
            if (!std::holds_alternative<ModeForCommand>(m_mode)) m_mode.emplace<ModeForCommand>();

            if constexpr (std::is_same_v<ModeForCommand, std::monostate>) {
                process_command(command);
            } else {
                process_command(command, std::get<ModeForCommand>(m_mode));
            }
        }

        auto receive(InBoundMessage const& message) -> void {
            // Ensure watchdog timer is reset and enabled now that we are receiving messages
            m_watchdog_timer->Instance->CNT = 0;
            if (!m_watchdog_enabled) {
                HAL_TIM_Base_Start(m_watchdog_timer);
                HAL_TIM_Base_Start_IT(m_watchdog_timer);
                m_watchdog_enabled = true;
            }

            m_inbound = message;
            process_command();
        }

        auto process_command() -> void {
            // m_elapsed_since_last_message = 0;
            std::visit([&](auto const& command) { process_command(command); }, m_inbound);
            drive_motor();
        }

        auto receive_watchdog_expired() -> void {
            HAL_TIM_Base_Stop(m_watchdog_timer);
            HAL_TIM_Base_Stop_IT(m_watchdog_timer);
            m_watchdog_enabled = false;

            // We lost connection or some other error happened
            // Make sure the motor stops
            m_inbound = IdleCommand{};
            process_command();
        }

        auto update() -> void {
            // 1. Update State
            update_limit_switches();

            update_encoder();

            process_command();

            // 2. Update Outbound Message (note we are not sending yet)
            ControllerDataState state{
                    .position = m_position.value_or(Radians{std::numeric_limits<float>::quiet_NaN()}),
                    .velocity = m_velocity.value_or(RadiansPerSecond{std::numeric_limits<float>::quiet_NaN()}),
                    .config_calib_error_data = ConfigCalibErrorInfo{
                            .configured = m_state_after_calib.has_value(),
                            .calibrated = m_state_after_calib.has_value(),
                            .error = m_error,
                    },
            };
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                SET_BIT_AT_INDEX(state.limit_switches.hit, i, m_limit_switches[i].pressed());
            }

            m_outbound = state;
        }

        auto send() -> void {
            m_fdcan.broadcast(m_outbound);
        }
    };

} // namespace mrover
