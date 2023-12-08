#pragma once

#include "../pch.hpp"

namespace mrover {

    template <typename T, auto Creater, auto Deleter>
    class SDLPointer
    {
        std::unique_ptr<T, decltype([](auto* p) { Deleter(p); })> mPointer;

        static auto check(T* result) -> T*
        {
            if (!result) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
            return result;
        }

    public:
        SDLPointer() = default;

        template <typename... Args>
        explicit SDLPointer(Args&&... args)
            : mPointer{check(Creater(std::forward<Args>(args)...))}
        {
        }

        [[nodiscard]] auto get() const noexcept -> T*
        {
            return mPointer.get();
        }
    };
}