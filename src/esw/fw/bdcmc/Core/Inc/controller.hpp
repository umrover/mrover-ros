#pragma once

#include <concepts>
#include <optional>
#include <variant>

#include "messaging.hpp"
#include "pidf.hpp"
#include "units.hpp"

template<typename T, typename TInput>
concept InputReader = requires(T t) {
    { t.read_input() } -> std::convertible_to<TInput>;
};

template<typename T, typename TOutput>
concept OutputWriter = requires(T t, TOutput output) {
    { t.write_output(output) };
};

template<typename TInput, typename TOutput,
         InputReader<TInput> Reader, OutputWriter<TOutput> Writer,
         typename TTime = seconds>
class Controller {
private:
    using Input = unit_t<TInput>;
    using Output = unit_t<TOutput>;
    using Time = unit_t<TTime>;

    struct PositionMode {
        PIDF<TInput, TOutput, TTime> pidf;
    };

    struct VelocityMode {
        PIDF<compound_unit<TInput, inverse<TTime>>, TOutput, TTime> pidf;
    };

    using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

    Reader m_reader;
    Writer m_writer;
    Message m_command;
    Mode m_mode;

    void feed(IdleCommand const& message, std::monostate) {
    }

    void feed(ThrottleCommand const& message, std::monostate) {
    }

    void feed(VelocityCommand const& message, VelocityMode mode) {
    }

    void feed(PositionCommand const& message, PositionMode mode) {
    }

    template<typename Command, typename V>
    struct command_to_mode;

    template<typename Command, typename ModeHead, typename... Modes>
    struct command_to_mode<Command, std::variant<ModeHead, Modes...>> {
        using type = std::conditional_t<
                requires(Controller controller, Command command, ModeHead mode) { controller.feed(command, mode); },
                ModeHead,
                typename command_to_mode<Command, std::variant<Modes...>>::type>;
    };

    template<typename Command>
    struct command_to_mode<Command, std::variant<>> {
        using type = std::monostate;
    };

    void feed(Message const& message) {
        std::visit(
                [&](auto const& command) {
                    // Find the feed function that has the right type for the command
                    using ModeForCommand = command_to_mode<decltype(command), Mode>::type;
                    // If the current mode is not the mode that the feed function expects, change the mode, providing a new blank mode
                    if (!std::holds_alternative<ModeForCommand>(m_mode))
                        m_mode.template emplace<ModeForCommand>();
                    feed(command, std::get<ModeForCommand>(m_mode));
                },
                message);
    }

public:
    void step(Message const& message) {
        feed(message);
    }
};
