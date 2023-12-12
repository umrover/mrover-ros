#pragma once

#include "pch.hpp"

#include "sdl_pointer.hpp"

using namespace std::literals;

namespace mrover
{
    // using uri_hash = std::size_t;

    constexpr static GLuint GL_INVALID_HANDLE = 0;

    class SimulatorNodelet;

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
        struct Binding
        {
            GLuint vao = GL_INVALID_HANDLE;
            GLuint vbo = GL_INVALID_HANDLE;
            GLuint ebo = GL_INVALID_HANDLE;
            GLsizei indicesCount{};
        };

        boost::container::static_vector<Binding, 4> bindings;

        explicit Mesh(std::string_view uri);

        ~Mesh();
    };

    struct URDF
    {
        urdf::Model model;

        URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init);
    };

    class SimulatorNodelet final : public nodelet::Nodelet
    {
        friend Mesh;
        friend URDF;

        // Settings

        Sint32 mQuitKey = SDL_SCANCODE_ESCAPE;
        Sint32 mRightKey = SDL_SCANCODE_D;
        Sint32 mLeftKey = SDL_SCANCODE_A;
        Sint32 mForwardKey = SDL_SCANCODE_W;
        Sint32 mBackwardKey = SDL_SCANCODE_S;
        Sint32 mUpKey = SDL_SCANCODE_SPACE;
        Sint32 mDownKey = SDL_SCANCODE_LCTRL;
        float mFlySpeed = 0.1f;

        // ROS

        ros::NodeHandle mNh, mPnh;

        // Rendering

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

        std::unordered_map<std::string, Mesh> mUriToMesh;

        Program mShaderProgram;

        // Physics

        std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
        std::unique_ptr<btCollisionDispatcher> mDispatcher;
        std::unique_ptr<btDbvtBroadphase> mOverlappingPairCache;
        std::unique_ptr<btSequentialImpulseConstraintSolver> mSolver;
        std::unique_ptr<btDiscreteDynamicsWorld> mPhysicsWorld;
        std::vector<std::unique_ptr<btCollisionShape>> mCollisionShapes;
        std::vector<std::unique_ptr<btCollisionObject>> mCollisionObjects;
        std::vector<std::unique_ptr<btMotionState>> mMotionStates;

        // Scene

        std::vector<URDF> mUrdfs;

        SE3 mCameraInWorld{R3{-2.0, 0.0, 0.0}, SO3{}};

        std::thread mRunThread;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override;

        auto initRender() -> void;

        auto initPhysics() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto freeLook() -> void;

        auto renderUrdf(URDF const& urdf) -> void;

        auto traverseLinkForRender(URDF const& urdf, urdf::LinkConstSharedPtr const& link) -> void;

        auto renderUpdate() -> void;

        auto physicsUpdate() -> void;
    };
} // namespace mrover
