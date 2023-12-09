#pragma once

#include "../pch.hpp"

#include "sdl_pointer.hpp"

namespace mrover
{
    struct Mesh
    {
        Mesh(std::filesystem::path const& path)
        {
        }
    };

    struct URDF
    {
        urdf::Model mModel;
        std::unordered_map<std::string, Mesh> link_meshes;

        explicit URDF(XmlRpc::XmlRpcValue const& init)
        {
            assert(xmlRpcValueToTypeOrDefault<std::string>(init, "type") == "urdf");

            auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
            mModel.initParam(paramName);

            for (auto const& [link_name, link] : mModel.links_)
            {
                assert(link->visual);
                assert(link->visual->geometry);
                assert(link->visual->geometry->type == urdf::Geometry::MESH);

                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
            }
        }
    };

    using Object = std::variant<URDF>;

    class SimulatorNodelet final : public nodelet::Nodelet
    {
        std::jthread mRunThread;

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
