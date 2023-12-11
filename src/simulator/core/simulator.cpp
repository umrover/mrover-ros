#include "simulator.hpp"

namespace mrover {

    constexpr int SDL_OK = 0;

    constexpr float DEG2RAD = std::numbers::pi_v<float> / 180.0f;

    static auto check(bool condition) -> void {
        if (!condition) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
    }

    auto SimulatorNodelet::parseParams() -> void {
        XmlRpc::XmlRpcValue objects;
        mNh.getParam("objects", objects);
        if (objects.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::invalid_argument{"objects must be an array"};

        for (int i = 0; i < objects.size(); ++i) {
            XmlRpc::XmlRpcValue const& object = objects[i];

            auto type = xmlRpcValueToTypeOrDefault<std::string>(object, "type");
            auto name = xmlRpcValueToTypeOrDefault<std::string>(object, "name", "<unnamed>");

            NODELET_INFO_STREAM(std::format("Loading object: {} of type: {}", name, type));

            if (type == "urdf") {
                mObjects.emplace_back(URDF{object});
            }
        }
    }

    SimulatorNodelet::~SimulatorNodelet() {
        mRunThread.join();
        // When you make an OpenGL context it binds to the thread that created it
        // This destructor is called from another thread, so we need to steal the context before the member destructors are run
        SDL_GL_MakeCurrent(mWindow.get(), mGlContext.get());
    }

    auto SimulatorNodelet::initRender() -> void {
        check(SDL_Init(SDL_INIT_VIDEO) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized SDL Version: {}.{}.{}", SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_PATCHLEVEL));

        SDL_DisplayMode displayMode;
        check(SDL_GetDesktopDisplayMode(0, &displayMode) == SDL_OK);
        assert(displayMode.w > 0 && displayMode.h > 0);

        auto w = static_cast<int>(displayMode.w * 0.8), h = static_cast<int>(displayMode.h * 0.8);
        mWindow = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{"MRover Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL};
        NODELET_INFO_STREAM(std::format("Created window of size: {}x{}", w, h));

        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1) == SDL_OK);
        mGlContext = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{mWindow.get()};
        check(SDL_GL_SetSwapInterval(1) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized OpenGL Version: {}.{}", reinterpret_cast<char const*>(glGetString(GL_VERSION)), reinterpret_cast<char const*>(glGetString(GL_SHADING_LANGUAGE_VERSION))));

        glewExperimental = GL_TRUE;
        check(glewInit() == GLEW_OK);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_LESS);

        auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
        mShaderProgram = {
                Shader{shadersPath / "pbr.vert", GL_VERTEX_SHADER},
                Shader{shadersPath / "pbr.frag", GL_FRAGMENT_SHADER}};
    }

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mRunThread = std::thread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto perspective(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f {
        // Equivalent to glm::perspectiveRH_NO
        float theta = fovY * 0.5f;
        float range = zFar - zNear;
        float invtan = 1.0f / tanf(theta);
        Eigen::Matrix4f result;
        result << invtan / aspect, 0.0f, 0.0f, 0.0f,
                0.0f, invtan, 0.0f, 0.0f,
                0.0f, 0.0f, -(zFar + zNear) / range, -2.0f * zFar * zNear / range,
                0.0f, 0.0f, -1.0f, 0.0f;
        return result;
    }

    auto SimulatorNodelet::traverseLinkForRender(URDF const& urdf, urdf::LinkConstSharedPtr const& link) -> void {
        assert(mShaderProgram.handle != GL_INVALID_HANDLE);
        glUseProgram(mShaderProgram.handle);

        GLint modelToWorldId = glGetUniformLocation(mShaderProgram.handle, "worldToCamera");
        assert(modelToWorldId != GL_INVALID_INDEX);
        GLint worldToCameraId = glGetUniformLocation(mShaderProgram.handle, "modelToWorld");
        assert(worldToCameraId != GL_INVALID_INDEX);
        GLint cameraToClipId = glGetUniformLocation(mShaderProgram.handle, "cameraToClip");
        assert(cameraToClipId != GL_INVALID_INDEX);

        Eigen::Matrix4f worldToCamera = mCameraInWorld.matrix().cast<float>();
        glUniform(worldToCameraId, worldToCamera);

        Eigen::Matrix4f modelToWorld = SE3{R3{}, SO3{}}.matrix().cast<float>();
        glUniform(modelToWorldId, modelToWorld);

        int w{}, h{};
        SDL_GetWindowSize(mWindow.get(), &w, &h);
        float aspect = static_cast<float>(w) / static_cast<float>(h);

        Eigen::Matrix4f cameraToClip = perspective(60.0f * DEG2RAD, aspect, 0.1f, 100.0f).cast<float>();
        glUniform(cameraToClipId, cameraToClip);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        if (link->visual && link->visual->geometry) {
            if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
                for (Mesh const& mesh = urdf.uriToMeshes.at(urdfMesh->filename);
                     auto const& [vao, _vbo, _ebo, indicesCount]: mesh.bindings) {
                    assert(vao != GL_INVALID_HANDLE);
                    assert(indicesCount > 0);

                    glBindVertexArray(vao);
                    glDrawElements(GL_TRIANGLES, indicesCount, GL_UNSIGNED_INT, nullptr);
                    // glBindVertexArray(0);
                }
            }
        }
        for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
            traverseLinkForRender(urdf, urdf.model.getLink(child_joint->child_link_name));
        }
    }

    auto SimulatorNodelet::renderObject(URDF const& urdf) -> void {
        assert(mShaderProgram.handle != GL_INVALID_HANDLE);

        traverseLinkForRender(urdf, urdf.model.getRoot());
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        int w{}, h{};
        SDL_GetWindowSize(mWindow.get(), &w, &h);

        glViewport(0, 0, w, h);
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (auto const& object: mObjects) {
            std::visit([&](auto const& o) { renderObject(o); }, object);
        }

        SDL_GL_SwapWindow(mWindow.get());
    }

    auto SimulatorNodelet::run() -> void try {

        initRender();

        parseParams();

        while (ros::ok()) {
            SDL_Event event;

            while (SDL_PollEvent(&event)) {
                switch (event.type) {
                    case SDL_KEYDOWN: {
                        constexpr float speed = 0.1f;
                        Sint32 key = event.key.keysym.sym;
                        if (key == quitKey) {
                            ros::requestShutdown();
                        } else if (key == rightKey) {
                            mCameraInWorld = SE3{R3{-speed, 0.0, 0}, SO3{}} * mCameraInWorld;
                        } else if (key == leftKey) {
                            mCameraInWorld = SE3{R3{speed, 0.0, 0}, SO3{}} * mCameraInWorld;
                        } else if (key == forwardKey) {
                            mCameraInWorld = SE3{R3{0.0, 0.0, speed}, SO3{}} * mCameraInWorld;
                        } else if (key == backwardKey) {
                            mCameraInWorld = SE3{R3{0.0, 0.0, -speed}, SO3{}} * mCameraInWorld;
                        } else if (key == upKey) {
                            mCameraInWorld = SE3{R3{0.0, speed, 0.0}, SO3{}} * mCameraInWorld;
                        } else if (key == downKey) {
                            mCameraInWorld = SE3{R3{0.0, -speed, 0.0}, SO3{}} * mCameraInWorld;
                        }
                    }
                    break;
                    case SDL_QUIT:
                        ros::requestShutdown();
                        break;
                    default:
                        break;
                }
            }

            renderUpdate();
        }

        // mObjects.clear();
        // { _ = std::move(mShaderProgram); }
        // { _ = std::move(mGlContext); }
        // { _ = std::move(mWindow); }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

} // namespace mrover
