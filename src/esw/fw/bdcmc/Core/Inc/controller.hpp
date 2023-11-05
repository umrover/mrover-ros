#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "config.hpp"
#include "hardware.hpp"
#include "messaging.hpp"
#include "pidf.hpp"
#include "units.hpp"

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
        FDCANBus m_fdcan_bus;

        InBoundMessage m_command;
        Mode m_mode;

        inline void force_configure() {
            check(m_config.configured(), Error_Handler);
        }

        void feed(IdleCommand const& message) {
            // TODO: Feel like we should be actively calling something here
            // maybe write_output with a throttle of 0?
        }

        void feed(ConfigCommand const& message) {
            m_config.configure(message);
        }

        void feed(ThrottleCommand const& message) {
            force_configure();

            m_writer.write(m_config, message.throttle);
        }

        void feed(VelocityCommand const& message, VelocityMode mode) {
            force_configure();

            auto [_, input] = m_reader.read(m_config);
            auto target = message.velocity;
            OutputUnit output = mode.pidf.calculate(input, target);
            m_writer.write(m_config, output);
        }

        void feed(PositionCommand const& message, PositionMode mode) {
            force_configure();

            auto [input, _] = m_reader.read(m_config);
            auto target = message.position;
            OutputUnit output = mode.pidf.calculate(input, target);
            m_writer.write(m_config, output);
        }

        void feed(EnableLimitSwitchesCommand const& message) {
            force_configure();

            // TODO: implement
        }

        void feed(AdjustCommand const& message) {
            force_configure();

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

        Controller(std::uint32_t id, Reader&& reader, Writer&& writer, FDCANBus const& fdcan_bus)
            : m_config{id}, m_reader{std::move(reader)}, m_writer{std::move(writer)}, m_fdcan_bus{fdcan_bus} {}

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

        void send() {
            /* Commented out for testing
            auto [position, velocity] = m_reader.read(m_config);
            m_fdcan_bus.broadcast(OutBoundMessage{ControllerDataState{
                    .position = position,
                    .velocity = velocity,
                    // TODO: Actually fill the config_calib_error_data with data
                    .config_calib_error_data = 0x00,
                    // TODO: Is this going to be right or left aligned?
                    // TODO: actually fill with correct values
                    .limit_switches = 0x00,
            }});
            */
           m_fdcan_bus.broadcast(OutBoundMessage{ControllerDataState{
                    .position = make_unit<Radians>(323323),
                    .velocity = make_unit<RadiansPerSecond>(323323),
                    .config_calib_error_data = 0x66,
                    .limit_switches = 0x66
           }});
        }
    };

} // namespace mrover
