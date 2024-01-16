#pragma once

// macOS does not have std::format yet
// So we define it and redirect it to fmt::format

#ifdef __APPLE__
#include <fmt/core.h>

namespace std {
    template<typename... Args>
    auto format(std::string_view format_str, Args&&... args) -> std::string {
        return fmt::format(format_str, std::forward<Args>(args)...);
    }
} // namespace std
#else
#include <format>
#endif
