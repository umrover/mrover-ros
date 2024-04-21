#pragma once

#include_next <poll.h>

#ifdef __APPLE__

// These are just stubs to make the code compile (NOT WORK) on macOS

inline auto ppoll(pollfd*, nfds_t, timespec const*, sigset_t const*) -> int {
    throw std::runtime_error("ppoll is not implemented on macOS");
}

#endif
