#pragma once

#include "pch.hpp"

namespace mrover {

    template<typename T, auto Creater, auto Deleter>
    class GLFWPointer {
        std::unique_ptr<T, decltype([](auto* p) { Deleter(p); })> mPointer;

        static auto check(T* result) -> T* {
            if (!result) throw std::runtime_error("GLFW Error");
            return result;
        }

    public:
        GLFWPointer() = default;

        template<typename... Args>
        explicit GLFWPointer(Args&&... args)
            : mPointer{check(Creater(std::forward<Args>(args)...))} {
        }

        [[nodiscard]] auto get() const noexcept -> T* {
            return mPointer.get();
        }
    };
} // namespace mrover