#pragma once

#include "../pch.hpp"

#include "sdl_pointer.hpp"

using namespace std::literals;

namespace mrover
{
    // using uri_hash = std::size_t;

    constexpr static GLuint GL_INVALID_HANDLE = 0;

    struct Shader
    {
        GLuint handle = GL_INVALID_HANDLE;

        Shader() = default;

        Shader(Shader&& other) noexcept;
        auto operator=(Shader&& other) noexcept -> Shader&;

        Shader(Shader& other) = delete;
        auto operator=(Shader& other) -> Shader& = delete;

        Shader(std::filesystem::path const& path, GLenum type);

        ~Shader();
    };

    struct Program
    {
        Shader vertexShader;
        Shader fragmentShader;

        GLuint handle = GL_INVALID_HANDLE;

        Program() = default;

        Program(Program&& other) noexcept;
        auto operator=(Program&& other) noexcept -> Program&;

        Program(Program& other) = delete;
        auto operator=(Program& other) -> Program& = delete;

        ~Program();

        Program(Shader&& vertexShader, Shader&& fragmentShader);
    };

    struct Mesh
    {
        GLuint vao = GL_INVALID_HANDLE;
        GLuint vbo = GL_INVALID_HANDLE;
        GLuint ebo = GL_INVALID_HANDLE;

        GLsizei vertexCount{}, indicesCount{};

        explicit Mesh(std::string_view uri);

        ~Mesh();
    };

    struct URDF
    {
        urdf::Model model;
        std::unordered_map<std::string, std::vector<Mesh>> uriToMeshes;

        explicit URDF(XmlRpc::XmlRpcValue const& init);
    };

    using Object = std::variant<URDF>;

    class SimulatorNodelet final : public nodelet::Nodelet
    {
        ros::NodeHandle mNh, mPnh;

        std::vector<Object> mObjects;

        Program mProgram;

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

        std::jthread mRunThread;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override = default;

        auto initRender() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto renderObject(URDF const& urdf) -> void;

        auto renderUpdate() -> void;
    };
} // namespace mrover
