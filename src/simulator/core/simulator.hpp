#pragma once

#include "../pch.hpp"

namespace mrover
{
    struct URDF
    {
        urdf::Model mModel;

        URDF(XmlRpc::XmlRpcValue const& init)
        {
            auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
            mModel.initParam(paramName);
        }
    };

    class SimulatorNodelet : public nodelet::Nodelet
    {
        std::jthread mThread;

        ros::NodeHandle mNh, mPnh;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override = default;

        auto onInit() -> void override;

        auto update() -> void;
    };

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
