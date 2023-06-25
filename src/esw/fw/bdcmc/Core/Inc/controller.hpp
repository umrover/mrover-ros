#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "messaging.hpp"
#include "pidf.hpp"
#include "units.hpp"

namespace mrover {

    template<typename T, typename TInput>
    concept InputReader = requires(T t) {
        { t.read_input() } -> std::convertible_to<TInput>;
    };

    template<typename T, typename TOutput>
    concept OutputWriter = requires(T t, TOutput output) {
        { t.write_output(output) };
    };

    template<Unitable Input, Unitable Output,
             InputReader<Input> Reader, OutputWriter<Output> Writer,
             Unitable Time = Seconds>
    class Controller {
    private:
        struct PositionMode {
            PIDF<Input, Output, Time> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<Input, inverse<Time>>, Output, Time> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        Reader m_reader;
        Writer m_writer;
        Message m_command;
        Mode m_mode;

        void feed(IdleCommand const& message) {
        }

        void feed(ThrottleCommand const& message) {
        }

        void feed(VelocityCommand const& message, VelocityMode mode) {
        }

        void feed(PositionCommand const& message, PositionMode mode) {
            mode.pidf.calculate(Input{}, Input{});
        }

        struct detail {
            template<typename Command, typename V>
            struct command_to_mode;

            template<typename Command, typename ModeHead, typename... Modes>
            struct command_to_mode<Command, std::variant<ModeHead, Modes...>> { // Linear search to find corresponding mode
                using type = std::conditional_t<
                        requires(Controller controller, Command command, ModeHead mode) { controller.feed(command, mode); },
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
        template<typename Command>
        void update(Command const& command) {
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

        void update(Message const& message) {
            std::visit([&](auto const& command) { update(command); }, message);
        }
    };

} // namespace mrover
