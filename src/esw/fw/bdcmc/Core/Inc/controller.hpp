#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "config.hpp"
#include "hardware.hpp"
#include "messaging.hpp"
#include "pidf.hpp"
#include "units/units.hpp"

namespace mrover {

    template<typename T, typename Input, typename TimeUnit = Seconds>
    concept InputReader = requires(T t, Config& config) {
        { t.read(config) } -> std::convertible_to<std::pair<Input, compound_unit<Input, inverse<TimeUnit>>>>;
    };

    template<typename T, typename Output>
    concept OutputWriter = requires(T t, Config& config, Output output) {
        { t.write(config, output) };
    };

    template<IsUnit InputUnit, IsUnit OutputUnit,
             InputReader<InputUnit> Reader, OutputWriter<OutputUnit> Writer,
             IsUnit TimeUnit = Seconds>
    class Controller {
    private:
        struct PositionMode {
            PIDF<InputUnit, OutputUnit, TimeUnit> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<InputUnit, inverse<TimeUnit>>, OutputUnit, TimeUnit> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        Config m_config;
        Reader m_reader;
        Writer m_writer;
        std::array<LimitSwitch, 4> m_limit_switches;
        FDCANBus m_fdcan_bus;

        Mode m_mode;

        bool m_should_limit_forward;
        bool m_should_limit_backward;
        bool m_is_calibrated;
        Radians m_current_position;
        Percent m_current_throttle;
        RadiansPerSecond m_current_velocity;
        // If the m_reader reads in 6 Radians and offset is 4 Radians,
        // Then my actual m_current_position should be 2 Radians.
        Radians m_offset_position;

        void write_output_if_valid(Percent output) {
        	if (m_should_limit_forward && output > Percent{0}) {
        		output = Percent{0};
			}
			else if (m_should_limit_backward && output < Percent{0}) {
				output = Percent{0};
			}
        	m_writer.write(m_config, output);
        	m_current_throttle = output;
        }

        void feed(AdjustCommand const& message) {
            // TODO - this needs to be implemented!
        }

        void feed(ConfigCommand const& message) {
            m_config.configure(message);  // TODO - this needs to be removed eventually
            // TODO - this referring to m_config being removed

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
                if (GET_BIT_AT_INDEX(m_config.limit_switch_info.present, i) && GET_BIT_AT_INDEX(m_config.limit_switch_info.enabled, i)) {
                    m_limit_switches[i].enable();
                }
            }
        }

        void feed(IdleCommand const& message) {
        }

        void feed(ThrottleCommand const& message) {
            if (!m_config.is_configured) {
                // TODO - set Error state
                return;
            }

            write_output_if_valid(message.throttle);
        }

        void feed(VelocityCommand const& message, VelocityMode mode) {
            if (!m_config.is_configured) {
                // TODO - set Error state
                return;
            }

            auto [_, input] = m_reader.read(m_config);
            auto target = message.velocity;
            OutputUnit output = mode.pidf.calculate(input, target);

            write_output_if_valid(output);
        }

        void feed(PositionCommand const& message, PositionMode mode) {
            if (!m_config.is_configured) {
                // TODO - set Error state
                return;
            }

            auto [input, _] = m_reader.read(m_config);
            auto target = message.position;
            OutputUnit output = mode.pidf.calculate(input, target);

            write_output_if_valid(output);
        }

        void feed(EnableLimitSwitchesCommand const& message) {
            // TODO: implement
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

        Controller(Reader&& reader, Writer&& writer, FDCANBus const& fdcan_bus)
            : m_reader{std::move(reader)}, m_writer{std::move(writer)}, m_fdcan_bus{fdcan_bus} {}

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

			auto [reader_position, m_current_velocity] = m_reader.read(m_config);
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
					|| (m_is_calibrated && m_current_position >= m_config.max_forward_pos));

			m_should_limit_backward = (std::ranges::any_of(m_limit_switches, [](
					LimitSwitch const& limit_switch) { return limit_switch.limit_backward(); })
					|| (m_is_calibrated && m_current_position <= m_config.max_backward_pos));

			write_output_if_valid(m_current_throttle);


            // 2. Send Information
            ConfigCalibErrorInfo config_calib_error_info;
            config_calib_error_info.configured = m_config.is_configured;
            config_calib_error_info.calibrated = m_is_calibrated; // TODO
            config_calib_error_info.error = 0;          // TODO

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
