#include "simulator.hpp"

namespace mrover {

    constexpr int SDL_OK = 0;

    static std::string const MESHES_PATH = "package://mrover/urdf/meshes/primitives";
    static std::string const CUBE_PRIMITIVE_URI = std::format("{}/cube.fbx", MESHES_PATH);
    static std::string const SPHERE_PRIMITIVE_URI = std::format("{}/sphere.fbx", MESHES_PATH);
    static std::string const CYLINDER_PRIMITIVE_URI = std::format("{}/cylinder.fbx", MESHES_PATH);

    static auto check(bool condition) -> void {
        if (!condition) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
    }

    auto btTransformToSim3(btTransform const& transform, btVector3 const& scale) -> SIM3 {
        btVector3 const& p = transform.getOrigin();
        btQuaternion const& q = transform.getRotation();
        return {R3{p.x(), p.y(), p.z()}, SO3{q.w(), q.x(), q.y(), q.z()}, R3{scale.x(), scale.y(), scale.z()}};
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

    auto SimulatorNodelet::initRender() -> void {
        // Force laptops with NVIDIA GPUs to use it instead of the integrated graphics
        setenv("DRI_PRIME", "1", true);
        setenv("__NV_PRIME_RENDER_OFFLOAD", "1", true);
        setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", true);

        check(SDL_Init(SDL_INIT_VIDEO) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized SDL Version: {}.{}.{}", SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_PATCHLEVEL));

        SDL_DisplayMode displayMode;
        check(SDL_GetDesktopDisplayMode(0, &displayMode) == SDL_OK);
        assert(displayMode.w > 0 && displayMode.h > 0);

        auto w = static_cast<int>(displayMode.w * 0.8), h = static_cast<int>(displayMode.h * 0.8);
#ifdef NDEBUG
        constexpr auto WINDOW_NAME = "MRover Simulator";
#else
        constexpr auto WINDOW_NAME = "MRover Simulator (DEBUG BUILD, MAY BE SLOW)";
#endif
        mWindow = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{WINDOW_NAME, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI};
        NODELET_INFO_STREAM(std::format("Created window of size: {}x{}", w, h));

        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24) == SDL_OK);
        // check(SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8) == SDL_OK);
        // check(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1) == SDL_OK);
        // check(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4) == SDL_OK);
        mGlContext = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{mWindow.get()};
        check(SDL_GL_SetSwapInterval(0) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized OpenGL Version: {}", reinterpret_cast<char const*>(glGetString(GL_VERSION))));
        NODELET_INFO_STREAM(std::format("\tShading Language Version: {}", reinterpret_cast<char const*>(glGetString(GL_SHADING_LANGUAGE_VERSION))));
        NODELET_INFO_STREAM(std::format("\tVendor: {}", reinterpret_cast<char const*>(glGetString(GL_VENDOR))));
        NODELET_INFO_STREAM(std::format("\tRenderer: {}", reinterpret_cast<char const*>(glGetString(GL_RENDERER))));

        glewExperimental = GL_TRUE;
        check(glewInit() == GLEW_OK);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        // float ddpi, hdpi, vdpi;
        // SDL_GetDisplayDPI(0, &ddpi, &hdpi, &vdpi);
        ImGuiIO& io = ImGui::GetIO();
        ImGuiStyle& style = ImGui::GetStyle();
        float scale = h > 1500 ? 2.0f : 1.0f;
        io.FontGlobalScale = scale;
        style.ScaleAllSizes(scale);
        ImGui_ImplSDL2_InitForOpenGL(mWindow.get(), mGlContext.get());
        ImGui_ImplOpenGL3_Init("#version 450 core");

        auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
        mPbrProgram = Program<2>{
                Shader{shadersPath / "pbr.vert", GL_VERTEX_SHADER},
                Shader{shadersPath / "pbr.frag", GL_FRAGMENT_SHADER},
        };
        glUseProgram(mPbrProgram.handle);
        mPbrProgram.uniform("lightInWorld", Eigen::Vector3f{0, 0, 5});
        mPbrProgram.uniform("lightColor", Eigen::Vector3f{1, 1, 1});

        mPointCloudProgram = Program<1>{
                Shader{shadersPath / "pc.comp", GL_COMPUTE_SHADER},
        };

        mUriToModel.emplace(CUBE_PRIMITIVE_URI, CUBE_PRIMITIVE_URI);
        mUriToModel.emplace(SPHERE_PRIMITIVE_URI, SPHERE_PRIMITIVE_URI);
        mUriToModel.emplace(CYLINDER_PRIMITIVE_URI, CYLINDER_PRIMITIVE_URI);
    }

    auto SimulatorNodelet::renderModel(Model& model, SIM3 const& modelToWorld, [[maybe_unused]] bool isRoverCamera) -> void {
        if (!model.areMeshesReady()) return;

        mPbrProgram.uniform("modelToWorld", modelToWorld.matrix().cast<float>());

        // See: http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/ for why this has to be treated specially
        // TLDR: it preserves orthogonality between normal vectors and their respective surfaces with any model scaling (including non-uniform)
        mPbrProgram.uniform("modelToWorldForNormals", modelToWorld.matrix().inverse().transpose().cast<float>());

        for (Model::Mesh& mesh: model.meshes) {
            if (mesh.vao == GL_INVALID_HANDLE) {
                glGenVertexArrays(1, &mesh.vao);
                assert(mesh.vao != GL_INVALID_HANDLE);
            }
            glBindVertexArray(mesh.vao);

            mesh.indices.prepare();
            if (mesh.vertices.prepare()) {
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.vertices)::value_type), nullptr);
                glEnableVertexAttribArray(0);
            }
            if (mesh.normals.prepare()) {
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.normals)::value_type), nullptr);
                glEnableVertexAttribArray(1);
            }
            if (mesh.uvs.prepare()) {
                glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.uvs)::value_type), nullptr);
                glEnableVertexAttribArray(2);
            }
            mesh.texture.prepare();

            if (mesh.texture.handle == GL_INVALID_HANDLE) {
                mPbrProgram.uniform("hasTexture", false);
                mPbrProgram.uniform("objectColor", Eigen::Vector3f{1, 1, 1});
            } else {
                mPbrProgram.uniform("hasTexture", true);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, mesh.texture.handle);
            }

            static_assert(std::is_same_v<decltype(mesh.indices)::value_type, std::uint32_t>);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.indices.data.size()), GL_UNSIGNED_INT, nullptr);
        }
    }

    auto SimulatorNodelet::renderModels(bool isRoverCamera) -> void {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        mPbrProgram.uniform("type", 1);

        for (auto const& [_, urdf]: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->visual && link->visual->geometry) {
                    if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
                        Model& model = mUriToModel.at(urdfMesh->filename);
                        btTransform linkInWorld = urdf.physics->getLink(urdf.linkNameToIndex.at(link->name)).m_cachedWorldTransform;
                        btTransform modelInLink = urdfPoseToBtTransform(link->visual->origin);
                        SIM3 worldToModel = btTransformToSim3(linkInWorld * modelInLink, btVector3{1, 1, 1});
                        renderModel(model, worldToModel, isRoverCamera);
                    }
                }
                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };

            renderLink(renderLink, urdf.model.getRoot());
        }
    }

    auto SimulatorNodelet::renderWireframeColliders() -> void {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mPbrProgram.uniform("type", 0);

        for (auto const& collider: mMultibodyCollider) {

            auto renderCollisionObject = [this](auto&& self, btTransform const& shapeToWorld, btCollisionShape const* shape) -> void {
                if (auto* box = dynamic_cast<btBoxShape const*>(shape)) {
                    btVector3 extents = box->getHalfExtentsWithoutMargin() * 2;
                    SIM3 worldToModel = btTransformToSim3(shapeToWorld, extents);
                    renderModel(mUriToModel.at(CUBE_PRIMITIVE_URI), worldToModel);
                } else if (auto* sphere = dynamic_cast<btSphereShape const*>(shape)) {
                    btScalar diameter = sphere->getRadius() * 2;
                    SIM3 modelToWorld = btTransformToSim3(shapeToWorld, btVector3{diameter, diameter, diameter});
                    renderModel(mUriToModel.at(SPHERE_PRIMITIVE_URI), modelToWorld);
                } else if (auto* cylinder = dynamic_cast<btCylinderShapeZ const*>(shape)) {
                    btVector3 extents = cylinder->getHalfExtentsWithoutMargin() * 2;
                    SIM3 modelToWorld = btTransformToSim3(shapeToWorld, extents);
                    renderModel(mUriToModel.at(CYLINDER_PRIMITIVE_URI), modelToWorld);
                } else if (auto* compound = dynamic_cast<btCompoundShape const*>(shape)) {
                    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
                        btTransform const& childToParent = compound->getChildTransform(i);
                        btCollisionShape const* childShape = compound->getChildShape(i);
                        btTransform childToWorld = shapeToWorld * childToParent;
                        self(self, childToWorld, childShape);
                    }
                } else if (auto* mesh = dynamic_cast<btBvhTriangleMeshShape const*>(shape)) {
                    SIM3 modelToWorld = btTransformToSim3(shapeToWorld, btVector3{1, 1, 1});
                    renderModel(mUriToModel.at(mMeshToUri.at(const_cast<btBvhTriangleMeshShape*>(mesh))), modelToWorld);
                } else if (dynamic_cast<btEmptyShape const*>(shape)) {
                } else {
                    NODELET_WARN_STREAM_ONCE(std::format("Tried to render unsupported collision shape: {}", shape->getName()));
                }
            };

            btTransform const& shapeToWorld = collider->getWorldTransform();
            btCollisionShape const* shape = collider->getCollisionShape();
            renderCollisionObject(renderCollisionObject, shapeToWorld, shape);
        }
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        int w{}, h{};
        SDL_GetWindowSize(mWindow.get(), &w, &h);

        glUseProgram(mPbrProgram.handle);

        float aspect = static_cast<float>(w) / static_cast<float>(h);
        Eigen::Matrix4f cameraToClip = perspective(mFov * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
        mPbrProgram.uniform("cameraToClip", cameraToClip);

        mPbrProgram.uniform("cameraInWorld", mCameraInWorld.position().cast<float>());
        Eigen::Matrix4f worldToCamera = ROS_TO_GL * mCameraInWorld.matrix().inverse().cast<float>();
        mPbrProgram.uniform("worldToCamera", worldToCamera);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (mRenderModels) renderModels(false);
        if (mRenderWireframeColliders) renderWireframeColliders();
    }

} // namespace mrover
