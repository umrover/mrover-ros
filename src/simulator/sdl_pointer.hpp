#pragma once

#include "pch.hpp"

namespace mrover {

    template<typename T, auto Creater, auto Deleter>
    class GlfwPointer {
        std::unique_ptr<T, decltype([](auto* p) { Deleter(p); })> mPointer;

        static auto check(T* result) -> T* {
            if (!result) throw std::runtime_error("GLFW Error");
            return result;
        }

    public:
        GlfwPointer() = default;

        template<typename... Args>
        explicit GlfwPointer(Args&&... args)
            : mPointer{check(Creater(std::forward<Args>(args)...))} {
        }

        [[nodiscard]] auto get() const noexcept -> T* {
            return mPointer.get();
        }
    };

    class GlfwInstance {
        bool mWasInitialized = false;

    public:
        GlfwInstance() = default;

        void init() {
            if (glfwInit() != GLFW_TRUE) throw std::runtime_error("Failed to initialize GLFW");

            mWasInitialized = true;
        }

        ~GlfwInstance() {
            if (mWasInitialized) glfwTerminate();
        }
    };

} // namespace mrover