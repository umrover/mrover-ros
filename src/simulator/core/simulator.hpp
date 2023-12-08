#pragma once

#include "../pch.hpp"

#include "sdl_pointer.hpp"

namespace mrover
{
    struct URDF
    {
        urdf::Model mModel;

        explicit URDF(XmlRpc::XmlRpcValue const& init)
        {
            assert(xmlRpcValueToTypeOrDefault<std::string>(init, "type") == "urdf");

            auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
            mModel.initParam(paramName);
        }
    };

    using Object = std::variant<URDF>;

    class SimulatorNodelet final : public nodelet::Nodelet
    {
        std::jthread mThread;

        ros::NodeHandle mNh, mPnh;

        std::vector<Object> mObjects;

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override = default;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto renderObject(URDF const& urdf) -> void;

        auto renderUpdate() -> void;
    };
}
