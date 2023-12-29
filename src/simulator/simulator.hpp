#pragma once

#include "pch.hpp"

#include "gl_objects.hpp"
#include "sdl_pointer.hpp"

using namespace std::literals;

namespace mrover {

    using Clock = std::chrono::high_resolution_clock;

    constexpr static float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

    // Convert from ROS's right-handed +x forward, +y left, +z up to OpenGL's right-handed +x right, +y up, +z backward
    static auto const ROS_TO_GL = (Eigen::Matrix4f{} << 0, -1, 0, 0, // OpenGL x = -ROS y
                                   0, 0, 1, 0,                       // OpenGL y = +ROS zp
                                   -1, 0, 0, 0,                      // OpenGL z = -ROS x
                                   0, 0, 0, 1)
                                          .finished();

    struct Camera;
    class SimulatorNodelet;

    struct Model {
        struct Mesh {
            GLuint vao = GL_INVALID_HANDLE; // Vertex array object

            SharedBuffer<Eigen::Vector3f, GL_ARRAY_BUFFER> vertices;
            SharedBuffer<Eigen::Vector3f, GL_ARRAY_BUFFER> normals;
            SharedBuffer<Eigen::Vector2f, GL_ARRAY_BUFFER> uvs;
            SharedBuffer<std::uint32_t, GL_ELEMENT_ARRAY_BUFFER> indices;
            SharedTexture texture;
        };

        // DO NOT access the mesh unless you are certain it has been set from the async loader
        std::vector<Mesh> meshes;
        // https://en.cppreference.com/w/cpp/thread/future
        std::future<decltype(meshes)> asyncMeshesLoader;

        explicit Model(std::string_view uri);

        auto waitMeshes() -> void;

        [[nodiscard]] auto areMeshesReady() -> bool;

        ~Model();
    };

    struct URDF {
        std::string name;
        urdf::Model model;

        URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init);

        auto makeColliderForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape*;

        auto makeCameraForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> Camera;
    };

    struct Camera {
        std::string linkName;
        cv::Size2i resolution;
        float rate;
        ros::Publisher pcPub;

        Clock::time_point lastUpdate = Clock::now();
        GLuint framebufferHandle = GL_INVALID_HANDLE;
        GLuint colorTextureHandle = GL_INVALID_HANDLE;
        GLuint depthTextureHandle = GL_INVALID_HANDLE;
        GLuint pointCloudArrayHandle = GL_INVALID_HANDLE;
    };

    class SimulatorNodelet final : public nodelet::Nodelet {
        friend Model;
        friend URDF;

        // Settings

        float mTargetUpdateRate = 180;
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
        Sint32 mToggleRenderModelsKey = SDLK_m;
        Sint32 mToggleRenderWireframeCollidersKey = SDLK_c;

        float mFlySpeed = 5.0f;
        float mRoverLinearSpeed = 1.0f;
        float mRoverAngularSpeed = 0.5f;
        float mLookSense = 0.004f;
        float mFov = 60.0f;
        btVector3 mGravityAcceleration{0.0f, 0.0f, -9.81f};
        bool mEnablePhysics = true;
        bool mRenderModels = true;
        bool mRenderWireframeColliders = false;

        float mFloat1 = 0.0f;
        float mFloat2 = 0.0f;

        // ROS

        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mTwistSub;

        ros::Publisher mPosePub;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        // Rendering

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

        std::unordered_map<std::string, Model> mUriToModel;

        Program<2> mPbrProgram;
        Program<1> mPointCloudProgram;

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

        // Scene

        std::vector<URDF> mUrdfs;

        SE3 mCameraInWorld{R3{-2.0, 0.0, 0.0}, SO3{}};

        static auto globalName(std::string_view modelName, std::string_view linkName) -> std::string {
            return std::format("{}#{}", modelName, linkName);
        }

        std::vector<Camera> mCameras;

        static constexpr float NEAR = 0.1f;
        static constexpr float FAR = 1000.0f;

        // Other

        std::thread mRunThread;

        LoopProfiler mLoopProfiler{"Simulator"};

        auto camerasUpdate(Camera& camera) -> void;

        auto gpsAndImusUpdate() -> void;

        auto linksToTfUpdate() -> void;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override;

        auto initRender() -> void;

        auto renderModel(Model& model, SIM3 const& modelToWorld, bool isRoverCamera = false) -> void;

        auto initPhysics() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto freeLook(Clock::duration dt, Uint8 const* keys) -> void;

        auto userControls(Clock::duration dt) -> void;

        auto renderModels(bool isRoverCamera) -> void;

        auto renderWireframeColliders() -> void;

        auto renderUpdate() -> void;

        auto physicsUpdate(Clock::duration dt) -> void;

        auto twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void;
    };

    auto uriToPath(std::string_view uri) -> std::filesystem::path;

    auto performXacro(std::filesystem::path const& path) -> std::string;

    auto readTextFile(std::filesystem::path const& path) -> std::string;

    auto readTexture(std::filesystem::path const& textureFileName) -> cv::Mat;

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform;

    auto btTransformToSim3(btTransform const& transform, btVector3 const& scale) -> SIM3;

    auto btTransformToSe3(btTransform const& transform) -> SE3;

    auto perspective(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f;

} // namespace mrover
