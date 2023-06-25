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

public:
    std::monostate feed(IdleCommand const& message, std::monostate) {
        return {};
    }

    std::monostate feed(ThrottleCommand const& message, std::monostate) {
        return {};
    }

    VelocityMode feed(VelocityCommand const& message, VelocityMode mode) {
        return mode;
    }

    PositionMode feed(PositionCommand const& message, PositionMode mode) {
        return mode;
    }

    void feed(Message const& message) {
        m_mode = std::visit(
                [&](auto const& command) {
                    // TODO: see if we can have void return type on feed's and take const reference for second argument
                    // Find the feed function that has the right type for the command
                    using ModeForCommand = decltype(feed(command, {}));
                    // If the current mode is not the mode that the feed function expects, change the mode, providing a new blank mode
                    if (!std::holds_alternative<ModeForCommand>(m_mode))
                        m_mode.template emplace<ModeForCommand>();
                    return Mode{feed(command, std::get<ModeForCommand>(m_mode))};
                },
                message);
    }

    void step(Message const& message) {
        feed(message);
    }
};
