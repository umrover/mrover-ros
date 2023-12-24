#pragma once

#include "pch.hpp"

#include "sdl_pointer.hpp"

using namespace std::literals;

namespace mrover {

    // using string_hash = std::size_t;

    constexpr static GLuint GL_INVALID_HANDLE = 0;

    class SimulatorNodelet;

    struct Shader {
        GLuint handle = GL_INVALID_HANDLE;

        Shader() = default;

        Shader(Shader&& other) noexcept;
        auto operator=(Shader&& other) noexcept -> Shader&;

        Shader(Shader& other) = delete;
        auto operator=(Shader& other) -> Shader& = delete;

        Shader(std::filesystem::path const& path, GLenum type);

        ~Shader();
    };

    struct Program {
        Shader vertexShader;
        Shader fragmentShader;
        std::unordered_map<std::string, GLint> uniforms;

        GLuint handle = GL_INVALID_HANDLE;

        Program() = default;

        Program(Program&& other) noexcept;
        auto operator=(Program&& other) noexcept -> Program&;

        Program(Program& other) = delete;
        auto operator=(Program& other) -> Program& = delete;

        ~Program();

        Program(Shader&& vertexShader, Shader&& fragmentShader);

        template<typename T>
        auto uniform(std::string const& name, T const& value) -> void {
            auto it = uniforms.find(name);
            if (it == uniforms.end()) {
                GLint location = glGetUniformLocation(handle, name.c_str());
                if (location == GL_INVALID_INDEX) throw std::runtime_error(std::format("Uniform {} not found", name));
                std::tie(it, std::ignore) = uniforms.emplace(name, location);
            }
            GLuint location = it->second;
            if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int>) {
                glUniform1i(location, value);
            } else {
                glUniform(location, value);
            }
        }
    };

    struct Model {
        struct Mesh {
            GLuint vao = GL_INVALID_HANDLE; // Vertex array object
            GLuint vbo = GL_INVALID_HANDLE; // Vertex buffer object
            GLuint nbo = GL_INVALID_HANDLE; // Normal buffer object
            GLuint ebo = GL_INVALID_HANDLE; // Element buffer object
            GLuint ubo = GL_INVALID_HANDLE; // UV-coordinate buffer object
            GLuint tbo = GL_INVALID_HANDLE; // Texture buffer object

            std::vector<Eigen::Vector3f> vertices;
            std::vector<Eigen::Vector3f> normals;
            std::vector<Eigen::Vector2f> uvs;
            std::vector<std::uint32_t> indices;
        };

        std::vector<Mesh> meshes;

        explicit Model(std::string_view uri);

        ~Model();
    };

    struct URDF {
        std::string name;
        urdf::Model model;

        URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init);
    };

    class SimulatorNodelet final : public nodelet::Nodelet {
        friend Model;
        friend URDF;

        // Settings

        float mTargetFps = 180;
        bool mHeadless = false;
        Sint32 mQuitKey = SDLK_q;
        Sint32 mInGuiKey = SDLK_ESCAPE;
        Sint32 mCamRightKey = SDL_SCANCODE_D;
        Sint32 mCamLeftKey = SDL_SCANCODE_A;
        Sint32 mCamForwardKey = SDL_SCANCODE_W;
        Sint32 mCamBackwardKey = SDL_SCANCODE_S;
        Sint32 mCamUpKey = SDL_SCANCODE_SPACE;
        Sint32 mCamDownKey = SDL_SCANCODE_LCTRL;
        Sint32 mRoverRightKey = SDL_SCANCODE_L;
        Sint32 mRoverLeftKey = SDL_SCANCODE_J;
        Sint32 mRoverForwardKey = SDL_SCANCODE_I;
        Sint32 mRoverBackwardKey = SDL_SCANCODE_COMMA;
        Sint32 mRoverStopKey = SDL_SCANCODE_K;
        Sint32 mTogglePhysicsKey = SDLK_p;

        float mFlySpeed = 5.0f;
        float mRoverLinearSpeed = 1.0f;
        float mRoverAngularSpeed = 0.5f;
        float mLookSense = 0.004f;
        float mFov = 60.0f;
        btVector3 mGravityAcceleration{0.0f, 0.0f, -9.81f};
        bool mEnablePhysics = false;
        bool mRenderModels = true;
        bool mRenderWireframeColliders = false;

        float mFloat1 = 0.0f;
        float mFloat2 = 0.0f;

        // ROS

        ros::NodeHandle mNh, mPnh;

        // std::vector<ros::Subscriber> mCanSubs;
        ros::Subscriber mTwistSub;

        // Rendering

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

        // TODO: this should use string hash
        std::unordered_map<std::string, Model> mUriToModel;

        Program mShaderProgram;

        bool mHasFocus = false;
        bool mInGui = false;

        // Physics

        std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
        std::unique_ptr<btCollisionDispatcher> mDispatcher;
        std::unique_ptr<btDbvtBroadphase> mOverlappingPairCache;
        std::unique_ptr<btConstraintSolver> mSolver;
        std::unique_ptr<btDiscreteDynamicsWorld> mDynamicsWorld;
        std::vector<std::unique_ptr<btCollisionObject>> mCollisionObjects;
        std::vector<std::unique_ptr<btCollisionShape>> mCollisionShapes;
        std::vector<std::unique_ptr<btMotionState>> mMotionStates;
        std::vector<std::unique_ptr<btTypedConstraint>> mConstraints;

        std::unordered_map<std::string, btRigidBody*> mLinkNameToRigidBody;
        std::unordered_map<std::string, btHingeConstraint*> mJointNameToHinges;
        std::unordered_map<std::string, btGeneric6DofSpring2Constraint*> mJointNameToSpringHinges;
        std::unordered_map<btBvhTriangleMeshShape*, std::string> mMeshToUri;

        template<typename T, typename... Args>
        auto makeBulletObject(auto& vector, Args&&... args) -> T* {
            auto pointer = std::make_unique<T>(std::forward<Args>(args)...);
            auto* rawPointer = pointer.get();
            vector.emplace_back(std::move(pointer));
            return rawPointer;
        }

        static auto globalName(std::string_view modelName, std::string_view linkName) -> std::string {
            return std::format("{}#{}", modelName, linkName);
        }

        // Scene

        std::vector<URDF> mUrdfs;

        SE3 mCameraInWorld{R3{-2.0, 0.0, 0.0}, SO3{}};

        std::thread mRunThread;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override;

        auto initRender() -> void;

        auto renderModel(Model const& model, SIM3 const& modelToWorld) -> void;

        auto initPhysics() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto freeLook(ros::Rate const& rate, Uint8 const* keys) -> void;

        auto userControls(ros::Rate const& rate) -> void;

        auto renderModels() -> void;

        auto renderWireframeColliders() -> void;

        auto renderUpdate() -> void;

        auto physicsUpdate(ros::Rate const& rate) -> void;

        auto twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void;
    };

    auto uriToPath(std::string_view uri) -> std::filesystem::path;

    auto performXacro(std::filesystem::path const& path) -> std::string;

    auto readTextFile(std::filesystem::path const& path) -> std::string;

    auto readTexture(std::filesystem::path const& textureFileName) -> cv::Mat;

} // namespace mrover
