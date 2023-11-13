#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "writer.hpp"
#include "reader.hpp"
#include "hardware.hpp"
#include "messaging.hpp"
#include "pidf.hpp"
#include "units/units.hpp"

namespace mrover {

    template<typename T, typename Input, typename TimeUnit = Seconds>
    concept IsReader = std::is_same_v<T, std::monostate> || requires(T t) {
        { t.read() } -> std::convertible_to<std::pair<Input, compound_unit<Input, inverse<TimeUnit>>>>;
    };

    template<typename T, typename Output>
    concept IsWriter = std::is_same_v<T, std::monostate> || requires(T t, Output output) {
        { t.write(output) };
    };

    using Writer = HBridgeWriter;

    using Reader = std::variant<
            std::monostate, FusedReader, QuadratureEncoderReader, AbsoluteEncoderReader>;

    template<IsUnit InputUnit = Radians, IsUnit OutputUnit = Percent, IsUnit TimeUnit = Seconds>
    class Controller {
    private:
        struct PositionMode {
            PIDF<InputUnit, OutputUnit, TimeUnit> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<InputUnit, inverse<TimeUnit>>, OutputUnit, TimeUnit> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        std::array<LimitSwitch, 4> m_limit_switches;
        FDCANBus m_fdcan_bus;

        Writer m_writer;
        Reader m_reader;
        Mode m_mode;

        bool m_should_limit_forward{};
        bool m_should_limit_backward{};
        bool m_is_calibrated{};
        Radians m_current_position{};
        Percent m_current_throttle{};
        RadiansPerSecond m_current_velocity{};
        // If the m_reader reads in 6 Radians and offset is 4 Radians,
        // Then my actual m_current_position should be 2 Radians.
        Radians m_offset_position{};
        bool m_is_configured{};
        Dimensionless m_gear_ratio{};
        Radians m_max_forward_pos;
        Radians m_max_backward_pos;

        I2C_HandleTypeDef* m_abs_enc_i2c{};

        BDCMCErrorInfo m_error{};

        std::optional<EncoderReading> read() {
            return std::visit([&](auto& reader) {
                if constexpr (std::is_same_v<decltype(reader), std::monostate&>) {
                	std::optional<EncoderReading> ret = std::nullopt;
                	return ret;
                }
                else {
                	return reader.read();
                }
            }, m_reader);
        }

        void write_output_if_valid(Percent output) {
        	if (m_should_limit_forward && output > 0_percent) {
        		output = 0_percent;
        		m_error = BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS;
			} else if (m_should_limit_backward && output < 0_percent) {
				output = 0_percent;
				m_error = BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS;
			} else {
				m_error = BDCMCErrorInfo::NO_ERROR;
			}
        	m_writer.write(output);
        	m_current_throttle = output;
        }

        void feed(AdjustCommand const& message) {
            auto reading = read();
            if (reading) {
                auto [reader_position, m_current_velocity] = *reading;
                m_is_calibrated = true;
                m_offset_position = reader_position - message.position;
                m_current_position = reader_position - m_offset_position;
            }
            // ELSE: If there is no reader, then we don't need to do anything
        }

        void feed(ConfigCommand const& message) {

        	m_is_configured = true;

			// Initialize values
			m_gear_ratio = message.gear_ratio;
			if (message.quad_abs_enc_info.quad_present && message.quad_abs_enc_info.abs_present) {
				Ratio quad_multiplier = (
						message.quad_abs_enc_info.quad_is_forward_polarity ? 1 : -1
								) * message.quad_enc_out_ratio;

				Ratio abs_multiplier = (
						message.quad_abs_enc_info.abs_is_forward_polarity ? 1 : -1
								) * message.abs_enc_out_ratio;

                m_reader = FusedReader(TIM4, m_abs_enc_i2c, quad_multiplier, abs_multiplier);

			}
			else if (message.quad_abs_enc_info.quad_present) {
				Ratio multiplier = (
						message.quad_abs_enc_info.quad_is_forward_polarity ? 1 : -1
								) * message.quad_enc_out_ratio;

                m_reader = QuadratureEncoderReader{
                	TIM4,
					multiplier};
			}
			else if (message.quad_abs_enc_info.abs_present) {
				Ratio multiplier = (
					message.quad_abs_enc_info.abs_is_forward_polarity ? 1 : -1
							) * message.abs_enc_out_ratio;

				// A1 and A2 are grounded
                m_reader = AbsoluteEncoderReader{SMBus{m_abs_enc_i2c}, 0, 0, multiplier};
			}

			m_writer.change_max_pwm(message.max_pwm);

			m_max_forward_pos = message.max_forward_pos;
			m_max_backward_pos = message.max_backward_pos;

            for (std::size_t i = 0; i < 4; ++i) {
            	if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i)) {
            		bool enabled = GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i);
            		bool active_high = GET_BIT_AT_INDEX(message.limit_switch_info.active_high, i);
            		bool used_for_readjustment = GET_BIT_AT_INDEX(message.limit_switch_info.use_for_readjustment, i);
            		bool limits_forward = GET_BIT_AT_INDEX(message.limit_switch_info.limits_forward, i);
            		Radians associated_position = Radians{message.limit_switch_info.limit_readj_pos[i]};

            		m_limit_switches[i].initialize(enabled, active_high, used_for_readjustment, limits_forward, associated_position);
            	}
            }

            for (std::size_t i = 0; i < 4; ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i) && GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i)) {
                    m_limit_switches[i].enable();
                }
            }
        }

        void feed(IdleCommand const& message) {
            // TODO - what is the expected behavior? just afk?
            if (!m_is_configured) {
            	m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            write_output_if_valid(0_percent);
        }

        void feed(ThrottleCommand const& message) {
            if (!m_is_configured) {
            	m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            write_output_if_valid(message.throttle);
        }

        void feed(VelocityCommand const& message, VelocityMode mode) {
            if (!m_is_configured) {
            	m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            auto reading = read();
            if (reading) {
                auto [_, input] = *reading;
                auto target = message.velocity;
                OutputUnit output = mode.pidf.calculate(input, target);

                write_output_if_valid(output);
            }
            else {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
            }

        }

        void feed(PositionCommand const& message, PositionMode mode) {
            if (!m_is_configured) {
            	m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }
            if (!m_is_calibrated) {
            	m_error = BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED;
            	return;
            }

            auto reading = read();
            if (reading) {
                auto [input, _] = *reading;
                auto target = message.position;
                OutputUnit output = mode.pidf.calculate(input, target);

                write_output_if_valid(output);
            }
            else {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
            }

        }

        void feed(EnableLimitSwitchesCommand const& message) {
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
                using type = std::conditional_t<requires(Controller controller, Command command, ModeHead mode) { controller.feed(command, mode); },
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

        Controller(TIM_HandleTypeDef* output_timer, std::array<LimitSwitch, 4> limit_switches, FDCANBus const& fdcan_bus, I2C_HandleTypeDef* abs_enc_i2c_line)
            : m_writer{HBridgeWriter(output_timer)},
              m_limit_switches{std::move(limit_switches)},
			  m_fdcan_bus{fdcan_bus},
              m_abs_enc_i2c{abs_enc_i2c_line},
			  m_error{BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED} {}

        template<typename Command>
        void process(Command const& command) {
            // Find the feed function that has the right type for the command
            using ModeForCommand = command_to_mode_t<Command, Mode>;

            // If the current mode is not the mode that the feed function expects, change the mode, providing a new blank mode
            if (!std::holds_alternative<ModeForCommand>(m_mode))
                m_mode.template emplace<ModeForCommand>();

            if constexpr (std::is_same_v<ModeForCommand, std::monostate>) {
                feed(command);
            } else {
                feed(command, std::get<ModeForCommand>(m_mode));
            }
        }

        void receive(InBoundMessage const& message) {
            std::visit([&](auto const& command) { process(command); }, message);
        }

        void update_and_send() {
            // 1. Update Information

            for (auto& limit_switch: m_limit_switches) {
                limit_switch.update_limit_switch();
            }

            auto reading = read();

            if (reading) {
                auto [reader_position, m_current_velocity] = *reading;

                for (auto& limit_switch: m_limit_switches) {
                    std::optional<Radians> readj_pos = limit_switch.get_readjustment_position();
                    if (readj_pos) {
                        m_is_calibrated = true;
                        m_offset_position = reader_position - readj_pos.value();
                    }
                }
                m_current_position = reader_position - m_offset_position;

                m_should_limit_forward = (std::ranges::any_of(m_limit_switches, [](
                        LimitSwitch const& limit_switch) { return limit_switch.limit_forward(); })
                        || (m_is_calibrated && m_current_position >= m_max_forward_pos));

                m_should_limit_backward = (std::ranges::any_of(m_limit_switches, [](
                        LimitSwitch const& limit_switch) { return limit_switch.limit_backward(); })
                        || (m_is_calibrated && m_current_position <= m_max_backward_pos));

                write_output_if_valid(m_current_throttle);

            }

            // 2. Send Information
            ConfigCalibErrorInfo config_calib_error_info;
            config_calib_error_info.configured = m_is_configured;
            config_calib_error_info.calibrated = m_is_calibrated;

			config_calib_error_info.error = static_cast<uint8_t>(m_error);

            LimitStateInfo limit_state_info;
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                SET_BIT_AT_INDEX(limit_state_info.hit, i, m_limit_switches[i].pressed());
            }

            m_fdcan_bus.broadcast(OutBoundMessage{ControllerDataState{
                    .position = m_current_position,
                    .velocity = m_current_velocity,
                    .config_calib_error_data = config_calib_error_info,
                    .limit_switches = limit_state_info,
            }});
        }
    };

} // namespace mrover
