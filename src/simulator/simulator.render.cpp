#define WEBGPU_CPP_IMPLEMENTATION

#include "simulator.hpp"

namespace mrover {

    static std::string const MESHES_PATH = "package://mrover/urdf/meshes/primitives";
    static std::string const CUBE_PRIMITIVE_URI = fmt::format("{}/cube.fbx", MESHES_PATH);
    static std::string const SPHERE_PRIMITIVE_URI = fmt::format("{}/sphere.fbx", MESHES_PATH);
    static std::string const CYLINDER_PRIMITIVE_URI = fmt::format("{}/cylinder.fbx", MESHES_PATH);

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

    auto SimulatorNodelet::initWindow() -> void {
#ifdef __linux__
        // Force laptops with NVIDIA GPUs to use it instead of the integrated graphics
        setenv("DRI_PRIME", "1", true);
        setenv("__NV_PRIME_RENDER_OFFLOAD", "1", true);
        setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", true);
#endif

        if (glfwInit() != GLFW_TRUE) throw std::runtime_error("Failed to initialize GLFW");
        glfwSetErrorCallback([](int error, char const* description) { throw std::runtime_error(fmt::format("GLFW Error {}: {}", error, description)); });
        NODELET_INFO_STREAM(fmt::format("Initialized GLFW Version: {}.{}.{}", GLFW_VERSION_MAJOR, GLFW_VERSION_MINOR, GLFW_VERSION_REVISION));

        int x, y, w, h;
        glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &x, &y, &w, &h);
#ifdef NDEBUG
        constexpr auto WINDOW_NAME = "MRover Simulator";
#else
        constexpr auto WINDOW_NAME = "MRover Simulator (DEBUG BUILD, MAY BE SLOW)";
#endif
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
        mWindow = GLFWPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow>{w, h, WINDOW_NAME, nullptr, nullptr};
        NODELET_INFO_STREAM(fmt::format("Created window of size: {}x{}", w, h));

        glfwSetWindowUserPointer(mWindow.get(), this);
        glfwSetKeyCallback(mWindow.get(), [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window));
            simulator->keyCallback(window, key, scancode, action, mods);
        });
    }

    auto SimulatorNodelet::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) -> void {
        if (key == mQuitKey) {
            ros::requestShutdown();
        }
        if (key == mTogglePhysicsKey) {
            mEnablePhysics = !mEnablePhysics;
        }
        if (key == mToggleRenderModelsKey) {
            mRenderModels = !mRenderModels;
        }
        if (key == mToggleRenderWireframeCollidersKey) {
            mRenderWireframeColliders = !mRenderWireframeColliders;
        }
        if (key == mInGuiKey) {
            mInGui = !mInGui;
        }
    }

    auto SimulatorNodelet::initRender() -> void {
        int w, h;
        glfwGetWindowSize(mWindow.get(), &w, &h);

        {
            wgpu::InstanceDescriptor descriptor{};
            mInstance.emplace(wgpu::createInstance(descriptor));
            if (!mInstance.value()) throw std::runtime_error("Failed to create WGPU instance");
        }
        {
            mSurface = glfwGetWGPUSurface(mInstance.value(), mWindow.get());
        }
        {
            wgpu::RequestAdapterOptions options;
            mAdapter = mInstance->requestAdapter(options);
            if (!mAdapter.value()) throw std::runtime_error("Failed to request WGPU adapter");
        }

        mDevice = mAdapter->createDevice();
        if (!mDevice.value()) throw std::runtime_error("Failed to create WGPU device");

        mQueue = mDevice->getQueue();
        if (!mQueue.value()) throw std::runtime_error("Failed to get WGPU queue");

        {
            wgpu::SwapChainDescriptor descriptor;
            descriptor.usage = wgpu::TextureUsage::RenderAttachment;
            descriptor.format = wgpu::TextureFormat::BGRA8Unorm;
            descriptor.width = w;
            descriptor.height = h;
            descriptor.presentMode = wgpu::PresentMode::Mailbox;
            mSwapChain = mDevice->createSwapChain(mSurface.value(), descriptor);
            if (!mSwapChain.value()) throw std::runtime_error("Failed to create WGPU swap chain");
        }
        {
            wgpu::ShaderModuleDescriptor shaderDescriptor;
            mPbrShaderModule = mDevice->createShaderModule(shaderDescriptor);
            if (!mPbrShaderModule.value()) throw std::runtime_error("Failed to create WGPU PBR shader module");

            wgpu::ShaderModuleWGSLDescriptor moduleDescriptor;
            auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
            std::string code = readTextFile(shadersPath / "shaders.wgsl");
            moduleDescriptor.code = code.c_str();
            moduleDescriptor.chain.sType = wgpu::SType::ShaderModuleWGSLDescriptor;

            shaderDescriptor.nextInChain = &moduleDescriptor.chain;
        }
        {
            wgpu::RenderPipelineDescriptor descriptor;
            descriptor.vertex.entryPoint = "vs_main";
            descriptor.vertex.module = mPbrShaderModule.value();
            descriptor.primitive.topology = wgpu::PrimitiveTopology::TriangleList;
            descriptor.primitive.stripIndexFormat = wgpu::IndexFormat::Uint32;
            descriptor.primitive.cullMode = wgpu::CullMode::Front;
            wgpu::FragmentState fragment;
            fragment.module = mPbrShaderModule.value();
            fragment.entryPoint = "fs_main";
            wgpu::BlendState blend;
            wgpu::ColorTargetState colorTarget;
            colorTarget.format = wgpu::TextureFormat::BGRA8Unorm;
            colorTarget.blend = &blend;
            colorTarget.writeMask = wgpu::ColorWriteMask::All;
            fragment.targetCount = 1;
            fragment.targets = &colorTarget;
            descriptor.fragment = &fragment;
            mPbrPipeline = mDevice->createRenderPipeline(descriptor);
            if (!mPbrPipeline.value()) throw std::runtime_error("Failed to create WGPU render pipeline");
        }

        // mPbrProgram = Program<2>{
        //         Shader{shadersPath / "pbr.vert", GL_VERTEX_SHADER},
        //         Shader{shadersPath / "pbr.frag", GL_FRAGMENT_SHADER},
        // };
        // glUseProgram(mPbrProgram.handle);
        // mPbrProgram.uniform("lightInWorld", Eigen::Vector3f{0, 0, 5});
        // mPbrProgram.uniform("lightColor", Eigen::Vector3f{1, 1, 1});

        // try {
        //     mPointCloudProgram = Program<1>{
        //             Shader{shadersPath / "pc.comp", GL_COMPUTE_SHADER},
        //     };
        // } catch (std::exception const& e) {
        //     NODELET_WARN_STREAM(e.what());
        // }

        mUriToModel.emplace(CUBE_PRIMITIVE_URI, CUBE_PRIMITIVE_URI);
        mUriToModel.emplace(SPHERE_PRIMITIVE_URI, SPHERE_PRIMITIVE_URI);
        mUriToModel.emplace(CYLINDER_PRIMITIVE_URI, CYLINDER_PRIMITIVE_URI);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForOther(mWindow.get(), true);
        ImGui_ImplWGPU_Init(mDevice.value(), 1, wgpu::TextureFormat::BGRA8Unorm, wgpu::TextureFormat::Undefined);

        ImGuiIO& io = ImGui::GetIO();
        ImGuiStyle& style = ImGui::GetStyle();
        float scale = h > 1500 ? 2.0f : 1.0f;
        io.FontGlobalScale = scale;
        style.ScaleAllSizes(scale);
    }

    auto SimulatorNodelet::renderModel(Model& model, SIM3 const& modelToWorld, [[maybe_unused]] bool isRoverCamera) -> void {
        if (!model.areMeshesReady()) return;

        // mPbrProgram.uniform("modelToWorld", modelToWorld.matrix().cast<float>());

        // // See: http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/ for why this has to be treated specially
        // // TLDR: it preserves orthogonality between normal vectors and their respective surfaces with any model scaling (including non-uniform)
        // mPbrProgram.uniform("modelToWorldForNormals", modelToWorld.matrix().inverse().transpose().cast<float>());

        // for (Model::Mesh& mesh: model.meshes) {
        //     if (mesh.vao == GL_INVALID_HANDLE) {
        //         glGenVertexArrays(1, &mesh.vao);
        //         assert(mesh.vao != GL_INVALID_HANDLE);
        //     }
        //     glBindVertexArray(mesh.vao);

        //     mesh.indices.prepare();
        //     if (mesh.vertices.prepare()) {
        //         glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.vertices)::value_type), nullptr);
        //         glEnableVertexAttribArray(0);
        //     }
        //     if (mesh.normals.prepare()) {
        //         glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.normals)::value_type), nullptr);
        //         glEnableVertexAttribArray(1);
        //     }
        //     if (mesh.uvs.prepare()) {
        //         glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(decltype(mesh.uvs)::value_type), nullptr);
        //         glEnableVertexAttribArray(2);
        //     }
        //     mesh.texture.prepare();

        //     if (mesh.texture.handle == GL_INVALID_HANDLE) {
        //         mPbrProgram.uniform("hasTexture", false);
        //         mPbrProgram.uniform("objectColor", Eigen::Vector3f{1, 1, 1});
        //     } else {
        //         mPbrProgram.uniform("hasTexture", true);
        //         glActiveTexture(GL_TEXTURE0);
        //         glBindTexture(GL_TEXTURE_2D, mesh.texture.handle);
        //     }

        //     static_assert(std::is_same_v<decltype(mesh.indices)::value_type, std::uint32_t>);
        //     glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.indices.data.size()), GL_UNSIGNED_INT, nullptr);
        // }
    }

    auto SimulatorNodelet::renderModels(bool isRoverCamera) -> void {
        // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // mPbrProgram.uniform("type", 1);

        // for (auto const& [_, urdf]: mUrdfs) {

        //     auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
        //         if (link->visual && link->visual->geometry) {
        //             if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
        //                 Model& model = mUriToModel.at(urdfMesh->filename);
        //                 SE3 linkInWorld = urdf.linkInWorld(link->name);
        //                 SE3 modelInLink = btTransformToSe3(urdfPoseToBtTransform(link->visual->origin));
        //                 SE3 modelInWorld = linkInWorld * modelInLink;
        //                 renderModel(model, SIM3{modelInWorld.position(), modelInWorld.rotation(), R3::Ones()}, isRoverCamera);
        //             }
        //         }
        //         for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
        //             self(self, urdf.model.getLink(child_joint->child_link_name));
        //         }
        //     };

        //     renderLink(renderLink, urdf.model.getRoot());
        // }
    }

    auto SimulatorNodelet::renderWireframeColliders() -> void {
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        // mPbrProgram.uniform("type", 0);

        // for (auto const& collider: mMultibodyCollider) {

        //     auto renderCollisionObject = [this](auto&& self, btTransform const& shapeToWorld, btCollisionShape const* shape) -> void {
        //         if (auto* box = dynamic_cast<btBoxShape const*>(shape)) {
        //             btVector3 extents = box->getHalfExtentsWithoutMargin() * 2;
        //             SIM3 worldToModel = btTransformToSim3(shapeToWorld, extents);
        //             renderModel(mUriToModel.at(CUBE_PRIMITIVE_URI), worldToModel);
        //         } else if (auto* sphere = dynamic_cast<btSphereShape const*>(shape)) {
        //             btScalar diameter = sphere->getRadius() * 2;
        //             SIM3 modelToWorld = btTransformToSim3(shapeToWorld, btVector3{diameter, diameter, diameter});
        //             renderModel(mUriToModel.at(SPHERE_PRIMITIVE_URI), modelToWorld);
        //         } else if (auto* cylinder = dynamic_cast<btCylinderShapeZ const*>(shape)) {
        //             btVector3 extents = cylinder->getHalfExtentsWithoutMargin() * 2;
        //             SIM3 modelToWorld = btTransformToSim3(shapeToWorld, extents);
        //             renderModel(mUriToModel.at(CYLINDER_PRIMITIVE_URI), modelToWorld);
        //         } else if (auto* compound = dynamic_cast<btCompoundShape const*>(shape)) {
        //             for (int i = 0; i < compound->getNumChildShapes(); ++i) {
        //                 btTransform const& childToParent = compound->getChildTransform(i);
        //                 btCollisionShape const* childShape = compound->getChildShape(i);
        //                 btTransform childToWorld = shapeToWorld * childToParent;
        //                 self(self, childToWorld, childShape);
        //             }
        //         } else if (auto* mesh = dynamic_cast<btBvhTriangleMeshShape const*>(shape)) {
        //             SIM3 modelToWorld = btTransformToSim3(shapeToWorld, btVector3{1, 1, 1});
        //             renderModel(mUriToModel.at(mMeshToUri.at(const_cast<btBvhTriangleMeshShape*>(mesh))), modelToWorld);
        //         } else if (dynamic_cast<btEmptyShape const*>(shape)) {
        //         } else {
        //             NODELET_WARN_STREAM_ONCE(fmt::format("Tried to render unsupported collision shape: {}", shape->getName()));
        //         }
        //     };

        //     btTransform const& shapeToWorld = collider->getWorldTransform();
        //     btCollisionShape const* shape = collider->getCollisionShape();
        //     renderCollisionObject(renderCollisionObject, shapeToWorld, shape);
        // }
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        // int w{}, h{};
        // SDL_GetWindowSize(mWindow.get(), &w, &h);

        // glUseProgram(mPbrProgram.handle);

        // float aspect = static_cast<float>(w) / static_cast<float>(h);
        // Eigen::Matrix4f cameraToClip = perspective(mFov * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
        // mPbrProgram.uniform("cameraToClip", cameraToClip);

        // mPbrProgram.uniform("cameraInWorld", mCameraInWorld.position().cast<float>());
        // Eigen::Matrix4f worldToCamera = ROS_TO_GL * mCameraInWorld.matrix().inverse().cast<float>();
        // mPbrProgram.uniform("worldToCamera", worldToCamera);

        // glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // glViewport(0, 0, w, h);
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // if (mRenderModels) renderModels(false);
        // if (mRenderWireframeColliders) renderWireframeColliders();

        wgpu::TextureView nextTexture = mSwapChain->getCurrentTextureView();
        if (!nextTexture) throw std::runtime_error("Failed to get WGPU next texture view");

        wgpu::RenderPassColorAttachment colorAttachment;
        colorAttachment.view = nextTexture;
        colorAttachment.loadOp = wgpu::LoadOp::Clear;
        colorAttachment.storeOp = wgpu::StoreOp::Store;
        colorAttachment.clearValue = {0.9f, 0.1f, 0.2f, 1.0f};

        wgpu::RenderPassDescriptor renderPassDescriptor;
        renderPassDescriptor.colorAttachmentCount = 1;
        renderPassDescriptor.colorAttachments = &colorAttachment;

        wgpu::CommandEncoder encoder = mDevice->createCommandEncoder();
        encoder.beginRenderPass(renderPassDescriptor);

        wgpu::CommandBuffer commands = encoder.finish();
        mQueue->submit(1, &commands);

        mSwapChain->present();
    }

} // namespace mrover
