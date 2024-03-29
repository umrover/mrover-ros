#pragma once

#include "pch.hpp"

namespace mrover {

    struct CanNetLink {

        CanNetLink() = default;

        explicit CanNetLink(std::string);

        CanNetLink(CanNetLink const&) = delete;
        CanNetLink& operator=(CanNetLink const&) = delete;

        CanNetLink(CanNetLink&& other) noexcept {
            *this = std::move(other);
        }

        CanNetLink& operator=(CanNetLink&& other) noexcept {
            mInterface = std::move(other.mInterface);
            mSocket = std::exchange(other.mSocket, nullptr);
            mCache = std::exchange(other.mCache, nullptr);
            return *this;
        }

        ~CanNetLink();

        std::string mInterface{};
        nl_cache* mCache{};
        nl_sock* mSocket{};
    };

} // namespace mrover
