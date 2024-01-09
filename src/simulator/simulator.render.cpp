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
        // Equivalent to glm::perspectiveLH_ZO
        // WGPU coordinate system is left-handed +x right, +y up, +z forward (same as DirectX)
        // Near and far clip planes correspond to Z values of 0 and +1 respectively (NDC)
        // See: https://www.w3.org/TR/webgpu/#coordinate-systems
        float theta = fovY * .5f;
        float range = zFar - zNear;
        float invtan = 1 / std::tan(theta);
        Eigen::Matrix4f result;
        result << invtan / aspect, 0, 0, 0,
                0, invtan, 0, 0,
                0, 0, zFar / range, -(2 * zFar * zNear) / range,
                0, 0, 1, 0;
        return result;
    }

    auto SimulatorNodelet::initWindow() -> void {
        // TODO(quintin): call glfwTerminate via raii
        mGlfwInstance.init();
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
        mWindow = GlfwPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow>{w, h, WINDOW_NAME, nullptr, nullptr};
        NODELET_INFO_STREAM(fmt::format("Created window of size: {}x{}", w, h));

        glfwSetWindowUserPointer(mWindow.get(), this);
        glfwSetKeyCallback(mWindow.get(), [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window));
            simulator->keyCallback(key, scancode, action, mods);
        });
        glfwSetFramebufferSizeCallback(mWindow.get(), [](GLFWwindow* window, int width, int height) {
            auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window));
            simulator->frameBufferResizedCallback(width, height);
        });
    }

    auto SimulatorNodelet::keyCallback(int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) -> void {
        if (action == GLFW_PRESS) {
            if (key == mQuitKey) ros::requestShutdown();
            if (key == mTogglePhysicsKey) mEnablePhysics = !mEnablePhysics;
            if (key == mToggleRenderModelsKey) mRenderModels = !mRenderModels;
            if (key == mToggleRenderWireframeCollidersKey) mRenderWireframeColliders = !mRenderWireframeColliders;
            if (key == mInGuiKey) mInGui = !mInGui;
        }
    }

    auto SimulatorNodelet::frameBufferResizedCallback(int width, int height) -> void {
        // TODO(quintin): resize framebuffer
        float aspect = static_cast<float>(width) / static_cast<float>(height);
        Eigen::Matrix4f cameraToClip = perspective(mFov * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
        mCameraToClipUniform.update(cameraToClip);
    }

    auto SimulatorNodelet::initRender() -> void {
        {
            wgpu::InstanceDescriptor descriptor{};
            mWgpuInstance = wgpu::createInstance(descriptor);
            if (!mWgpuInstance) throw std::runtime_error("Failed to create WGPU instance");
        }
        {
            mSurface = glfwGetWGPUSurface(mWgpuInstance, mWindow.get());
        }
        {
            wgpu::RequestAdapterOptions options;
            options.powerPreference = wgpu::PowerPreference::HighPerformance; // Request that laptops use the discrete GPU if available
            mAdapter = mWgpuInstance.requestAdapter(options);
            if (!mAdapter) throw std::runtime_error("Failed to request WGPU adapter");

            wgpu::AdapterProperties properties;
            mAdapter.getProperties(&properties);

            ROS_INFO_STREAM(fmt::format("\tWGPU Adapter Name: {}", properties.name));
            ROS_INFO_STREAM(fmt::format("\tWGPU Adapter Vendor: {}", properties.vendorName));
            ROS_INFO_STREAM(fmt::format("\tWGPU Adapter Driver: {}", properties.driverDescription));
        }

        mDevice = mAdapter.createDevice();
        if (!mDevice) throw std::runtime_error("Failed to create WGPU device");

        mErrorCallback = mDevice.setUncapturedErrorCallback([](wgpu::ErrorType type, char const* message) {
            ROS_ERROR_STREAM(fmt::format("WGPU Error {}: {}", static_cast<int>(type), message));
        });

        mQueue = mDevice.getQueue();
        if (!mQueue) throw std::runtime_error("Failed to get WGPU queue");

        {
            int width, height;
            glfwGetFramebufferSize(mWindow.get(), &width, &height);

            wgpu::SwapChainDescriptor descriptor;
            descriptor.usage = wgpu::TextureUsage::RenderAttachment;
            descriptor.format = wgpu::TextureFormat::BGRA8Unorm;
            descriptor.width = width;
            descriptor.height = height;
            descriptor.presentMode = wgpu::PresentMode::Mailbox;
            mSwapChain = mDevice.createSwapChain(mSurface, descriptor);
            if (!mSwapChain) throw std::runtime_error("Failed to create WGPU swap chain");
        }
        {
            wgpu::ShaderModuleWGSLDescriptor shaderSourceDescriptor;
            auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
            std::string code = readTextFile(shadersPath / "shaders.wgsl");
            shaderSourceDescriptor.code = code.c_str();
            shaderSourceDescriptor.chain.sType = wgpu::SType::ShaderModuleWGSLDescriptor;

            wgpu::ShaderModuleDescriptor shaderDescriptor;
            shaderDescriptor.nextInChain = &shaderSourceDescriptor.chain;

            mPbrShaderModule = mDevice.createShaderModule(shaderDescriptor);
            if (!mPbrShaderModule) throw std::runtime_error("Failed to create WGPU PBR shader module");
        }
        {
            wgpu::RenderPipelineDescriptor descriptor;
            descriptor.label = "PBR";
            descriptor.primitive.topology = wgpu::PrimitiveTopology::TriangleList;
            descriptor.primitive.cullMode = wgpu::CullMode::Back;
            descriptor.multisample.count = 1;
            descriptor.multisample.mask = 0xFFFFFFFF;

            // wgpu::DepthStencilState depthStencil;
            // depthStencil.depthCompare = wgpu::CompareFunction::Less;
            // depthStencil.depthWriteEnabled = true;
            // depthStencil.format = wgpu::TextureFormat::Depth32Float;
            // descriptor.depthStencil = &depthStencil;

            {
                std::array<wgpu::VertexAttribute, 3> attributes{};
                attributes[0].format = wgpu::VertexFormat::Float32x3;
                attributes[0].shaderLocation = 0;
                attributes[1].format = wgpu::VertexFormat::Float32x3;
                attributes[1].shaderLocation = 1;
                attributes[2].format = wgpu::VertexFormat::Float32x2;
                attributes[2].shaderLocation = 2;
                std::array<wgpu::VertexBufferLayout, 3> vertexBufferLayout{};
                vertexBufferLayout[0].arrayStride = sizeof(float) * 3;
                vertexBufferLayout[0].stepMode = wgpu::VertexStepMode::Vertex;
                vertexBufferLayout[0].attributeCount = 1;
                vertexBufferLayout[0].attributes = attributes.data() + 0;
                vertexBufferLayout[1].arrayStride = sizeof(float) * 3;
                vertexBufferLayout[1].stepMode = wgpu::VertexStepMode::Vertex;
                vertexBufferLayout[1].attributeCount = 1;
                vertexBufferLayout[1].attributes = attributes.data() + 1;
                vertexBufferLayout[2].arrayStride = sizeof(float) * 2;
                vertexBufferLayout[2].stepMode = wgpu::VertexStepMode::Vertex;
                vertexBufferLayout[2].attributeCount = 1;
                vertexBufferLayout[2].attributes = attributes.data() + 2;

                descriptor.vertex.entryPoint = "vs_main";
                descriptor.vertex.module = mPbrShaderModule;
                descriptor.vertex.bufferCount = vertexBufferLayout.size();
                descriptor.vertex.buffers = vertexBufferLayout.data();
            }

            std::array<wgpu::BindGroupLayoutEntry, 4> vertexBindGroupLayoutEntires{};
            vertexBindGroupLayoutEntires[0].binding = 0;
            vertexBindGroupLayoutEntires[0].visibility = wgpu::ShaderStage::Vertex;
            vertexBindGroupLayoutEntires[0].buffer.type = wgpu::BufferBindingType::Uniform;
            vertexBindGroupLayoutEntires[0].buffer.minBindingSize = sizeof(Eigen::Matrix4f);
            vertexBindGroupLayoutEntires[1].binding = 1;
            vertexBindGroupLayoutEntires[1].visibility = wgpu::ShaderStage::Vertex;
            vertexBindGroupLayoutEntires[1].buffer.type = wgpu::BufferBindingType::Uniform;
            vertexBindGroupLayoutEntires[1].buffer.minBindingSize = sizeof(Eigen::Matrix4f);
            vertexBindGroupLayoutEntires[2].binding = 2;
            vertexBindGroupLayoutEntires[2].visibility = wgpu::ShaderStage::Vertex;
            vertexBindGroupLayoutEntires[2].buffer.type = wgpu::BufferBindingType::Uniform;
            vertexBindGroupLayoutEntires[2].buffer.minBindingSize = sizeof(Eigen::Matrix4f);
            vertexBindGroupLayoutEntires[3].binding = 3;
            vertexBindGroupLayoutEntires[3].visibility = wgpu::ShaderStage::Vertex;
            vertexBindGroupLayoutEntires[3].buffer.type = wgpu::BufferBindingType::Uniform;
            vertexBindGroupLayoutEntires[3].buffer.minBindingSize = sizeof(Eigen::Matrix4f);

            wgpu::BindGroupLayoutDescriptor vertexBindGroupLayoutDescriptor;
            vertexBindGroupLayoutDescriptor.entryCount = vertexBindGroupLayoutEntires.size();
            vertexBindGroupLayoutDescriptor.entries = vertexBindGroupLayoutEntires.data();
            wgpu::BindGroupLayout vertexBindGroupLayout = mDevice.createBindGroupLayout(vertexBindGroupLayoutDescriptor);


            wgpu::FragmentState fragment;
            fragment.module = mPbrShaderModule;
            fragment.entryPoint = "fs_main";
            wgpu::BlendState blend;
            blend.color.srcFactor = wgpu::BlendFactor::SrcAlpha;
            blend.color.dstFactor = wgpu::BlendFactor::OneMinusSrcAlpha;
            blend.color.operation = wgpu::BlendOperation::Add;
            blend.alpha.srcFactor = wgpu::BlendFactor::Zero;
            blend.alpha.dstFactor = wgpu::BlendFactor::One;
            blend.alpha.operation = wgpu::BlendOperation::Add;
            wgpu::ColorTargetState colorTarget;
            colorTarget.format = wgpu::TextureFormat::BGRA8Unorm;
            colorTarget.blend = &blend;
            colorTarget.writeMask = wgpu::ColorWriteMask::All;
            fragment.targetCount = 1;
            fragment.targets = &colorTarget;
            descriptor.fragment = &fragment;

            // std::array<wgpu::BindGroupLayoutEntry, 1> fragmentBindGroupLayoutEntires{};
            // fragmentBindGroupLayoutEntires[0].binding = 0;
            // fragmentBindGroupLayoutEntires[0].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[0].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[0].buffer.minBindingSize = sizeof(std::uint32_t);
            // fragmentBindGroupLayoutEntires[1].binding = 1;
            // fragmentBindGroupLayoutEntires[1].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[1].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[1].buffer.minBindingSize = sizeof(std::uint32_t);
            // fragmentBindGroupLayoutEntires[2].binding = 2;
            // fragmentBindGroupLayoutEntires[2].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[2].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[2].buffer.minBindingSize = sizeof(Eigen::Vector3f);
            // fragmentBindGroupLayoutEntires[3].binding = 3;
            // fragmentBindGroupLayoutEntires[3].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[3].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[3].buffer.minBindingSize = sizeof(Eigen::Vector3f);
            // fragmentBindGroupLayoutEntires[4].binding = 4;
            // fragmentBindGroupLayoutEntires[4].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[4].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[4].buffer.minBindingSize = sizeof(Eigen::Vector3f);
            // fragmentBindGroupLayoutEntires[5].binding = 5;
            // fragmentBindGroupLayoutEntires[5].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[5].buffer.type = wgpu::BufferBindingType::Uniform;
            // fragmentBindGroupLayoutEntires[5].buffer.minBindingSize = sizeof(Eigen::Vector3f);
            // fragmentBindGroupLayoutEntires[6].binding = 6;
            // fragmentBindGroupLayoutEntires[6].visibility = wgpu::ShaderStage::Fragment;
            // fragmentBindGroupLayoutEntires[6].sampler.type = wgpu::SamplerBindingType::Filtering;
            //
            // wgpu::BindGroupLayoutDescriptor fragmentBindGroupLayoutDescriptor;
            // fragmentBindGroupLayoutDescriptor.entryCount = vertexBindGroupLayoutEntires.size();
            // fragmentBindGroupLayoutDescriptor.entries = vertexBindGroupLayoutEntires.data();
            // wgpu::BindGroupLayout fragmentBindGroupLayout = mDevice.createBindGroupLayout(fragmentBindGroupLayoutDescriptor);


            wgpu::PipelineLayoutDescriptor pipelineLayoutDescriptor;
            // std::array<WGPUBindGroupLayout, 2> bindGroupLayouts{vertexBindGroupLayout, fragmentBindGroupLayout};
            std::array<WGPUBindGroupLayout, 1> bindGroupLayouts{vertexBindGroupLayout};
            pipelineLayoutDescriptor.bindGroupLayoutCount = bindGroupLayouts.size();
            pipelineLayoutDescriptor.bindGroupLayouts = bindGroupLayouts.data();
            descriptor.layout = mDevice.createPipelineLayout(pipelineLayoutDescriptor);

            mPbrPipeline = mDevice.createRenderPipeline(descriptor);
            if (!mPbrPipeline) throw std::runtime_error("Failed to create WGPU render pipeline");
        }
        {
            mModelToWorldUniform.init(mDevice, mQueue, "modelToWorld");
            mWorldToCameraUniform.init(mDevice, mQueue, "worldToCamera");
            mCameraToClipUniform.init(mDevice, mQueue, "cameraToClip");
            mModelToWorldForNormalsUniform.init(mDevice, mQueue, "modelToWorldForNormals");

            std::array<wgpu::BindGroupEntry, 4> entries{};
            entries[0].binding = 0;
            entries[0].buffer = mModelToWorldUniform.buffer;
            entries[0].offset = 0;
            entries[0].size = sizeof(Eigen::Matrix4f);
            entries[1].binding = 1;
            entries[1].buffer = mWorldToCameraUniform.buffer;
            entries[1].offset = 0;
            entries[1].size = sizeof(Eigen::Matrix4f);
            entries[2].binding = 2;
            entries[2].buffer = mCameraToClipUniform.buffer;
            entries[2].offset = 0;
            entries[2].size = sizeof(Eigen::Matrix4f);
            entries[3].binding = 3;
            entries[3].buffer = mModelToWorldForNormalsUniform.buffer;
            entries[3].offset = 0;
            entries[3].size = sizeof(Eigen::Matrix4f);

            wgpu::BindGroupDescriptor descriptor;
            descriptor.layout = mPbrPipeline.getBindGroupLayout(0);
            descriptor.entryCount = entries.size();
            descriptor.entries = entries.data();
            mVertexBindGroup = mDevice.createBindGroup(descriptor);
        }
        // {
        //     mMaterialUniform.init(mDevice, mQueue);
        //
        //     std::array<wgpu::BindGroupEntry, 1> entries{};
        //     entries[0].binding = 0;
        //     entries[0].buffer = mMaterialUniform.buffer;
        //     entries[0].offset = 0;
        //     entries[0].size = sizeof(std::uint32_t);
        //
        //     wgpu::BindGroupDescriptor descriptor;
        //     descriptor.layout = mPbrPipeline.getBindGroupLayout(1);
        //     descriptor.entryCount = entries.size();
        //     descriptor.entries = entries.data();
        //     mFragmentBindGroup = mDevice.createBindGroup(descriptor);
        // }

        mUriToModel.try_emplace(CUBE_PRIMITIVE_URI, CUBE_PRIMITIVE_URI);
        mUriToModel.try_emplace(SPHERE_PRIMITIVE_URI, SPHERE_PRIMITIVE_URI);
        mUriToModel.try_emplace(CYLINDER_PRIMITIVE_URI, CYLINDER_PRIMITIVE_URI);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForOther(mWindow.get(), true);
        ImGui_ImplWGPU_Init(mDevice, 1, wgpu::TextureFormat::BGRA8Unorm, wgpu::TextureFormat::Undefined);

        int x, y, w, h;
        glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &x, &y, &w, &h);

        ImGuiIO& io = ImGui::GetIO();
        ImGuiStyle& style = ImGui::GetStyle();
        float scale = h > 1500 ? 2.0f : 1.0f;
        io.FontGlobalScale = scale;
        style.ScaleAllSizes(scale);
    }

    auto SimulatorNodelet::renderModel(Model& model, SIM3 const& modelToWorld, [[maybe_unused]] bool isRoverCamera) -> void {
        if (!model.areMeshesReady()) return;

        mModelToWorldUniform.update(modelToWorld.matrix().cast<float>());

        // // See: http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/ for why this has to be treated specially
        // // TLDR: it preserves orthogonality between normal vectors and their respective surfaces with any model scaling (including non-uniform)
        mModelToWorldForNormalsUniform.update(modelToWorld.matrix().inverse().transpose().cast<float>());

        for (Model::Mesh& mesh: model.meshes) {
            mesh.indices.ensureUploaded(mDevice, mQueue, wgpu::BufferUsage::Index);
            mesh.vertices.ensureUploaded(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.normals.ensureUploaded(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.uvs.ensureUploaded(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.texture.prepare();

            mRenderPass.setVertexBuffer(0, mesh.vertices.buffer, 0, mesh.vertices.sizeBytes());
            mRenderPass.setVertexBuffer(1, mesh.normals.buffer, 0, mesh.normals.sizeBytes());
            mRenderPass.setVertexBuffer(2, mesh.uvs.buffer, 0, mesh.uvs.sizeBytes());
            mRenderPass.setIndexBuffer(mesh.indices.buffer, wgpu::IndexFormat::Uint32, 0, mesh.indices.sizeBytes());

            // if (mesh.texture.handle == GL_INVALID_HANDLE) {
            //     mPbrProgram.uniform("hasTexture", false);
            //     mPbrProgram.uniform("objectColor", Eigen::Vector3f{1, 1, 1});
            // } else {
            //     mPbrProgram.uniform("hasTexture", true);
            //     glActiveTexture(GL_TEXTURE0);
            //     glBindTexture(GL_TEXTURE_2D, mesh.texture.handle);
            // }

            static_assert(std::is_same_v<decltype(mesh.indices)::value_type, std::uint32_t>);
            mRenderPass.drawIndexed(mesh.indices.data.size(), 1, 0, 0, 0);
        }
    }

    auto SimulatorNodelet::renderModels() -> void {
        // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // mMaterialUniform.update(1);

        for (auto const& [_, urdf]: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->visual && link->visual->geometry) {
                    if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
                        Model& model = mUriToModel.at(urdfMesh->filename);
                        SE3 linkInWorld = urdf.linkInWorld(link->name);
                        SE3 modelInLink = btTransformToSe3(urdfPoseToBtTransform(link->visual->origin));
                        SE3 modelInWorld = linkInWorld * modelInLink;
                        renderModel(model, SIM3{modelInWorld.position(), modelInWorld.rotation(), R3::Ones()});
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
        wgpu::TextureView nextTexture = mSwapChain.getCurrentTextureView();
        if (!nextTexture) throw std::runtime_error("Failed to get WGPU next texture view");

        wgpu::RenderPassColorAttachment colorAttachment;
        colorAttachment.view = nextTexture;
        colorAttachment.loadOp = wgpu::LoadOp::Clear;
        colorAttachment.storeOp = wgpu::StoreOp::Store;
        colorAttachment.clearValue = {0.1f, 0.1f, 0.1f, 1.0f};

        wgpu::RenderPassDescriptor renderPassDescriptor;
        renderPassDescriptor.colorAttachmentCount = 1;
        renderPassDescriptor.colorAttachments = &colorAttachment;

        wgpu::CommandEncoder encoder = mDevice.createCommandEncoder();
        mRenderPass = encoder.beginRenderPass(renderPassDescriptor);
        {
            mRenderPass.setPipeline(mPbrPipeline);
            mRenderPass.setBindGroup(0, mVertexBindGroup, 0, nullptr);
            // mRenderPass.setBindGroup(1, mFragmentBindGroup, 0, nullptr);
            camerasUpdate();
            {
                // mCameraInWorldUniform.update(mCameraInWorld.position().cast<float>());
                Eigen::Matrix4f worldToCamera = ROS_TO_WGPU * mCameraInWorld.matrix().inverse().cast<float>();
                mWorldToCameraUniform.update(worldToCamera);

                if (mRenderModels) renderModels();
                if (mRenderWireframeColliders) renderWireframeColliders();
            }
        }
        guiUpdate();

        mRenderPass.end();

        wgpu::CommandBuffer commands = encoder.finish();
        mQueue.submit(commands);

        mSwapChain.present();
    }

} // namespace mrover
