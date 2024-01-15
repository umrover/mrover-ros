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

    auto SimulatorNodelet::makeTextureAndView(int width, int height, wgpu::TextureFormat const& format, wgpu::TextureUsage const& usage, wgpu::TextureAspect const& aspect) -> std::pair<wgpu::Texture, wgpu::TextureView> {
        wgpu::TextureDescriptor textureDescriptor;
        textureDescriptor.dimension = wgpu::TextureDimension::_2D;
        textureDescriptor.format = format;
        textureDescriptor.mipLevelCount = 1;
        textureDescriptor.sampleCount = 1;
        textureDescriptor.size = {static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), 1};
        textureDescriptor.usage = usage;
        textureDescriptor.viewFormatCount = 1;
        textureDescriptor.viewFormats = reinterpret_cast<WGPUTextureFormat const*>(&format);
        wgpu::Texture texture = mDevice.createTexture(textureDescriptor);
        if (!texture) throw std::runtime_error{"Failed to make WGPU texture"};

        wgpu::TextureViewDescriptor textureViewDescriptor;
        textureViewDescriptor.aspect = aspect;
        textureViewDescriptor.arrayLayerCount = 1;
        textureViewDescriptor.mipLevelCount = 1;
        textureViewDescriptor.dimension = wgpu::TextureViewDimension::_2D;
        textureViewDescriptor.format = format;
        wgpu::TextureView textureView = texture.createView(textureViewDescriptor);
        if (!textureView) throw std::runtime_error{"Failed to make WGPU texture view"};

        return {texture, textureView};
    }

    auto SimulatorNodelet::makeFramebuffers(int width, int height) -> void {
        wgpu::SwapChainDescriptor descriptor;
        descriptor.usage = wgpu::TextureUsage::RenderAttachment;
        descriptor.format = COLOR_FORMAT;
        descriptor.width = width;
        descriptor.height = height;
        descriptor.presentMode = wgpu::PresentMode::Immediate;
        mSwapChain = mDevice.createSwapChain(mSurface, descriptor);
        if (!mSwapChain) throw std::runtime_error("Failed to create WGPU swap chain");
        std::tie(mDepthTexture, mDepthTextureView) = makeTextureAndView(width, height, DEPTH_FORMAT, wgpu::TextureUsage::RenderAttachment, wgpu::TextureAspect::DepthOnly);
    }

    auto computeCameraToClip(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f {
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
        return result * ROS_TO_WGPU;
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
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
        mWindow = GlfwPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow>{w, h, WINDOW_NAME, nullptr, nullptr};
        NODELET_INFO_STREAM(fmt::format("Created window of size: {}x{}", w, h));

        if (glfwRawMouseMotionSupported()) glfwSetInputMode(mWindow.get(), GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

        glfwSetWindowUserPointer(mWindow.get(), this);
        glfwSetKeyCallback(mWindow.get(), [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            if (auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window))) {
                simulator->keyCallback(key, scancode, action, mods);
            }
        });
        glfwSetFramebufferSizeCallback(mWindow.get(), [](GLFWwindow* window, int width, int height) {
            if (auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window))) {
                simulator->frameBufferResizedCallback(width, height);
            }
        });
        glfwSetWindowFocusCallback(mWindow.get(), [](GLFWwindow* window, int focused) {
            if (auto* simulator = static_cast<SimulatorNodelet*>(glfwGetWindowUserPointer(window))) {
                simulator->mHasFocus = focused;
                if (focused && !simulator->mInGui) simulator->centerCursor();
            }
        });
        glfwSetWindowCloseCallback(mWindow.get(), [](GLFWwindow*) {
            ros::requestShutdown();
        });
    }

    auto SimulatorNodelet::frameBufferResizedCallback(int width, int height) -> void {
        if (width == 0 || height == 0) return;

        mQueue.submit(0, nullptr);

        mDepthTextureView.release();
        mDepthTexture.destroy();
        mDepthTexture.release();
        mSwapChain.release();

        makeFramebuffers(width, height);
    }

    auto SimulatorNodelet::initRender() -> void {
        {
            wgpu::InstanceDescriptor descriptor{};
            mWgpuInstance = wgpu::createInstance(descriptor);
            if (!mWgpuInstance) throw std::runtime_error("Failed to create WGPU instance");
        }
        {
            mSurface = glfwGetWGPUSurface(mWgpuInstance, mWindow.get());
            if (!mSurface) throw std::runtime_error("Failed to create WGPU surface");
        }
        {
            wgpu::RequestAdapterOptions options;
            options.powerPreference = wgpu::PowerPreference::HighPerformance; // Request that laptops use the discrete GPU if available
            options.compatibleSurface = mSurface;
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

        int width, height;
        glfwGetFramebufferSize(mWindow.get(), &width, &height);

        makeFramebuffers(width, height);

        {
            wgpu::ShaderModuleWGSLDescriptor shaderSourceDescriptor;
            auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
            std::string code = readTextFile(shadersPath / "shaders.wgsl");
            shaderSourceDescriptor.code = code.c_str();
            shaderSourceDescriptor.chain.sType = wgpu::SType::ShaderModuleWGSLDescriptor;

            wgpu::ShaderModuleDescriptor shaderDescriptor;
            shaderDescriptor.nextInChain = &shaderSourceDescriptor.chain;

            mShaderModule = mDevice.createShaderModule(shaderDescriptor);
            if (!mShaderModule) throw std::runtime_error("Failed to create WGPU PBR shader module");
        }
        {
            wgpu::RenderPipelineDescriptor descriptor;
            descriptor.label = "PBR";
            descriptor.primitive.topology = wgpu::PrimitiveTopology::TriangleList;
            descriptor.primitive.cullMode = wgpu::CullMode::Back;
            descriptor.multisample.count = 1;
            descriptor.multisample.mask = 0xFFFFFFFF;

            wgpu::DepthStencilState depthStencil;
            depthStencil.depthCompare = wgpu::CompareFunction::Less;
            depthStencil.depthWriteEnabled = true;
            depthStencil.format = DEPTH_FORMAT;
            depthStencil.stencilFront.compare = wgpu::CompareFunction::Always;
            depthStencil.stencilBack.compare = wgpu::CompareFunction::Always;
            descriptor.depthStencil = &depthStencil;

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
                descriptor.vertex.module = mShaderModule;
                descriptor.vertex.bufferCount = vertexBufferLayout.size();
                descriptor.vertex.buffers = vertexBufferLayout.data();
            }

            wgpu::FragmentState fragment;
            fragment.module = mShaderModule;
            fragment.entryPoint = "fs_main";
            wgpu::BlendState blend;
            blend.color.srcFactor = wgpu::BlendFactor::SrcAlpha;
            blend.color.dstFactor = wgpu::BlendFactor::OneMinusSrcAlpha;
            blend.color.operation = wgpu::BlendOperation::Add;
            blend.alpha.srcFactor = wgpu::BlendFactor::Zero;
            blend.alpha.dstFactor = wgpu::BlendFactor::One;
            blend.alpha.operation = wgpu::BlendOperation::Add;
            wgpu::ColorTargetState colorTarget;
            colorTarget.format = COLOR_FORMAT;
            colorTarget.blend = &blend;
            colorTarget.writeMask = wgpu::ColorWriteMask::All;
            fragment.targetCount = 1;
            fragment.targets = &colorTarget;
            descriptor.fragment = &fragment;

            std::array<wgpu::BindGroupLayoutEntry, 3> meshBindGroupLayoutEntries{};
            meshBindGroupLayoutEntries[0].binding = 0;
            meshBindGroupLayoutEntries[0].visibility = wgpu::ShaderStage::Fragment | wgpu::ShaderStage::Vertex;
            meshBindGroupLayoutEntries[0].buffer.type = wgpu::BufferBindingType::Uniform;
            meshBindGroupLayoutEntries[0].buffer.minBindingSize = sizeof(ModelUniforms);
            meshBindGroupLayoutEntries[1].binding = 1;
            meshBindGroupLayoutEntries[1].visibility = wgpu::ShaderStage::Fragment;
            meshBindGroupLayoutEntries[1].texture.sampleType = wgpu::TextureSampleType::Float;
            meshBindGroupLayoutEntries[1].texture.viewDimension = wgpu::TextureViewDimension::_2D;
            meshBindGroupLayoutEntries[2].binding = 2;
            meshBindGroupLayoutEntries[2].visibility = wgpu::ShaderStage::Fragment;
            meshBindGroupLayoutEntries[2].sampler.type = wgpu::SamplerBindingType::Filtering;

            wgpu::BindGroupLayoutDescriptor meshBindGroupLayourDescriptor;
            meshBindGroupLayourDescriptor.entryCount = meshBindGroupLayoutEntries.size();
            meshBindGroupLayourDescriptor.entries = meshBindGroupLayoutEntries.data();
            wgpu::BindGroupLayout meshBindGroupLayout = mDevice.createBindGroupLayout(meshBindGroupLayourDescriptor);

            wgpu::BindGroupLayoutEntry sceneBindGroupLayoutEntry;
            sceneBindGroupLayoutEntry.binding = 0;
            sceneBindGroupLayoutEntry.visibility = wgpu::ShaderStage::Vertex | wgpu::ShaderStage::Fragment;
            sceneBindGroupLayoutEntry.buffer.type = wgpu::BufferBindingType::Uniform;
            sceneBindGroupLayoutEntry.buffer.minBindingSize = sizeof(SceneUniforms);

            wgpu::BindGroupLayoutDescriptor sceneBindGroupLayourDescriptor;
            sceneBindGroupLayourDescriptor.entryCount = 1;
            sceneBindGroupLayourDescriptor.entries = &sceneBindGroupLayoutEntry;
            wgpu::BindGroupLayout sceneBindGroupLayout = mDevice.createBindGroupLayout(sceneBindGroupLayourDescriptor);


            wgpu::PipelineLayoutDescriptor pipelineLayoutDescriptor;
            std::array<WGPUBindGroupLayout, 2> bindGroupLayouts{meshBindGroupLayout, sceneBindGroupLayout};
            pipelineLayoutDescriptor.bindGroupLayoutCount = bindGroupLayouts.size();
            pipelineLayoutDescriptor.bindGroupLayouts = bindGroupLayouts.data();
            descriptor.layout = mDevice.createPipelineLayout(pipelineLayoutDescriptor);

            mPbrPipeline = mDevice.createRenderPipeline(descriptor);
            if (!mPbrPipeline) throw std::runtime_error("Failed to create WGPU render pipeline");
        }
        {
            std::array<wgpu::BindGroupLayoutEntry, 4> bindGroupLayoutEntries{};
            bindGroupLayoutEntries[0].binding = 0;
            bindGroupLayoutEntries[0].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[0].buffer.type = wgpu::BufferBindingType::Uniform;
            bindGroupLayoutEntries[0].buffer.minBindingSize = sizeof(ComputeUniforms);
            bindGroupLayoutEntries[1].binding = 1;
            bindGroupLayoutEntries[1].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[1].texture.sampleType = wgpu::TextureSampleType::Float;
            bindGroupLayoutEntries[1].texture.viewDimension = wgpu::TextureViewDimension::_2D;
            bindGroupLayoutEntries[2].binding = 2;
            bindGroupLayoutEntries[2].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[2].texture.sampleType = wgpu::TextureSampleType::Depth;
            bindGroupLayoutEntries[2].texture.viewDimension = wgpu::TextureViewDimension::_2D;
            bindGroupLayoutEntries[3].binding = 3;
            bindGroupLayoutEntries[3].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[3].buffer.type = wgpu::BufferBindingType::Storage;
            bindGroupLayoutEntries[3].buffer.minBindingSize = sizeof(Point);

            wgpu::BindGroupLayoutDescriptor bindGroupLayoutDescriptor;
            bindGroupLayoutDescriptor.entryCount = bindGroupLayoutEntries.size();
            bindGroupLayoutDescriptor.entries = bindGroupLayoutEntries.data();
            wgpu::BindGroupLayout bindGroupLayout = mDevice.createBindGroupLayout(bindGroupLayoutDescriptor);

            wgpu::PipelineLayoutDescriptor layoutDescriptor;
            layoutDescriptor.bindGroupLayoutCount = 1;
            layoutDescriptor.bindGroupLayouts = reinterpret_cast<WGPUBindGroupLayout const*>(&bindGroupLayout);
            wgpu::PipelineLayout layout = mDevice.createPipelineLayout(layoutDescriptor);

            wgpu::ComputePipelineDescriptor descriptor;
            descriptor.compute.entryPoint = "cs_main";
            descriptor.compute.module = mShaderModule;
            descriptor.layout = layout;
            mPointCloudPipeline = mDevice.createComputePipeline(descriptor);
        }

        mUriToModel.try_emplace(CUBE_PRIMITIVE_URI, CUBE_PRIMITIVE_URI);
        mUriToModel.try_emplace(SPHERE_PRIMITIVE_URI, SPHERE_PRIMITIVE_URI);
        mUriToModel.try_emplace(CYLINDER_PRIMITIVE_URI, CYLINDER_PRIMITIVE_URI);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForOther(mWindow.get(), true);
        ImGui_ImplWGPU_Init(mDevice, 1, COLOR_FORMAT, DEPTH_FORMAT);

        int x, y, w, h;
        glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &x, &y, &w, &h);

        ImGuiIO& io = ImGui::GetIO();
        ImGuiStyle& style = ImGui::GetStyle();
        float scale = h > 1500 ? 2.0f : 1.0f;
        io.FontGlobalScale = scale;
        style.ScaleAllSizes(scale);
    }

    auto SimulatorNodelet::renderModel(wgpu::RenderPassEncoder& pass, Model& model, Uniform<ModelUniforms>& uniforms, SIM3 const& modelToWorld, [[maybe_unused]] bool isRoverCamera) -> void {
        if (!model.areMeshesReady()) return;

        if (!uniforms.buffer) uniforms.init(mDevice);
        uniforms.value.modelToWorld = modelToWorld.matrix().cast<float>();
        // See: http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/ for why this has to be treated specially
        // TLDR: it preserves orthogonality between normal vectors and their respective surfaces with any model scaling (including non-uniform)
        uniforms.value.modelToWorldForNormals = modelToWorld.matrix().inverse().transpose().cast<float>();
        uniforms.value.material = 1;
        uniforms.enqueueWrite();

        for (Model::Mesh& mesh: model.meshes) {
            mesh.indices.enqueueWriteIfUnitialized(mDevice, mQueue, wgpu::BufferUsage::Index);
            mesh.vertices.enqueueWriteIfUnitialized(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.normals.enqueueWriteIfUnitialized(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.uvs.enqueueWriteIfUnitialized(mDevice, mQueue, wgpu::BufferUsage::Vertex);
            mesh.texture.enqueWriteIfUnitialized(mDevice);

            std::array<wgpu::BindGroupEntry, 3> bindGroupEntires{};
            bindGroupEntires[0].binding = 0;
            bindGroupEntires[0].buffer = uniforms.buffer;
            bindGroupEntires[0].size = sizeof(ModelUniforms);
            bindGroupEntires[1].binding = 1;
            bindGroupEntires[1].textureView = mesh.texture.view;
            bindGroupEntires[2].binding = 2;
            bindGroupEntires[2].sampler = mesh.texture.sampler;
            wgpu::BindGroupDescriptor descriptor;
            descriptor.layout = mPbrPipeline.getBindGroupLayout(0);
            descriptor.entryCount = bindGroupEntires.size();
            descriptor.entries = bindGroupEntires.data();
            wgpu::BindGroup bindGroup = mDevice.createBindGroup(descriptor);

            pass.setBindGroup(0, bindGroup, 0, nullptr);
            pass.setVertexBuffer(0, mesh.vertices.buffer, 0, mesh.vertices.sizeBytes());
            pass.setVertexBuffer(1, mesh.normals.buffer, 0, mesh.normals.sizeBytes());
            pass.setVertexBuffer(2, mesh.uvs.buffer, 0, mesh.uvs.sizeBytes());
            pass.setIndexBuffer(mesh.indices.buffer, wgpu::IndexFormat::Uint32, 0, mesh.indices.sizeBytes());

            static_assert(std::is_same_v<decltype(mesh.indices)::value_type, std::uint32_t>);
            pass.drawIndexed(mesh.indices.data.size(), 1, 0, 0, 0);

            bindGroup.release();
        }
    }

    auto SimulatorNodelet::renderModels(wgpu::RenderPassEncoder& encoder) -> void {
        for (auto& [_, urdf]: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->visual && link->visual->geometry) {
                    if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
                        Model& model = mUriToModel.at(urdfMesh->filename);
                        SE3 linkInWorld = urdf.linkInWorld(link->name);
                        SE3 modelInLink = btTransformToSe3(urdfPoseToBtTransform(link->visual->origin));
                        SE3 modelInWorld = linkInWorld * modelInLink;
                        renderModel(encoder, model, urdf.linkNameToUniform[link->name], SIM3{modelInWorld.position(), modelInWorld.rotation(), R3::Ones()});
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

    // TODO(quintin): Free pointers in here at the end of the progrma
    boost::container::static_vector<sensor_msgs::PointCloud2*, 32> pointCloudPool;

    auto getPooledPointCloud() -> sensor_msgs::PointCloud2* {
        if (pointCloudPool.empty()) return new sensor_msgs::PointCloud2{};

        auto* pointCloud = pointCloudPool.back();
        pointCloudPool.pop_back();
        return pointCloud;
    }

    auto returnPooledPointCloud(sensor_msgs::PointCloud2* pointCloud) -> void {
        pointCloudPool.push_back(pointCloud);
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        wgpu::TextureView nextTexture = mSwapChain.getCurrentTextureView();
        if (!nextTexture) throw std::runtime_error("Failed to get WGPU next texture view");

        wgpu::RenderPassColorAttachment colorAttachment;
        colorAttachment.loadOp = wgpu::LoadOp::Clear;
        colorAttachment.storeOp = wgpu::StoreOp::Store;
        colorAttachment.clearValue = {0.1f, 0.1f, 0.1f, 1.0f};

        wgpu::RenderPassDepthStencilAttachment depthStencilAttachment;
        depthStencilAttachment.depthClearValue = 1.0f;
        depthStencilAttachment.depthLoadOp = wgpu::LoadOp::Clear;
        depthStencilAttachment.depthStoreOp = wgpu::StoreOp::Store;
        depthStencilAttachment.stencilLoadOp = wgpu::LoadOp::Undefined;
        depthStencilAttachment.stencilStoreOp = wgpu::StoreOp::Undefined;
        depthStencilAttachment.stencilReadOnly = true;

        wgpu::RenderPassDescriptor renderPassDescriptor;
        renderPassDescriptor.colorAttachmentCount = 1;
        renderPassDescriptor.colorAttachments = &colorAttachment;
        renderPassDescriptor.depthStencilAttachment = &depthStencilAttachment;

        wgpu::CommandEncoder encoder = mDevice.createCommandEncoder();
        {
            for (Camera& camera: mCameras) {
                if (camera.callback) {
                    mWgpuInstance.processEvents();
                    if (camera.pointCloudBufferStaging.getMapState() == wgpu::BufferMapState::Mapped) {
                        auto pointCloud = boost::shared_ptr<sensor_msgs::PointCloud2>{getPooledPointCloud(), &returnPooledPointCloud};
                        pointCloud->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
                        pointCloud->is_dense = true;
                        pointCloud->width = camera.resolution.x();
                        pointCloud->height = camera.resolution.y();
                        pointCloud->header.stamp = ros::Time::now();
                        pointCloud->header.frame_id = "zed2i_left_camera_frame";
                        fillPointCloudMessageHeader(pointCloud);

                        auto* fromCompute = camera.pointCloudBufferStaging.getConstMappedRange(0, camera.pointCloudBufferStaging.getSize());
                        auto* toMessage = pointCloud->data.data();
                        std::memcpy(toMessage, fromCompute, camera.pointCloudBufferStaging.getSize());
                        camera.pointCloudBufferStaging.unmap();
                        camera.callback = nullptr;

                        camera.pcPub.publish(pointCloud);
                    }
                }
                if (!camera.callback && camera.updateTask.shouldUpdate()) {
                    // TODO(quintin): Move these into camera update
                    colorAttachment.view = camera.colorTextureView;
                    depthStencilAttachment.view = camera.depthTextureView;

                    cameraUpdate(camera, encoder, renderPassDescriptor);

                    camera.thisUpdate = true;
                }
            }
        }
        {
            colorAttachment.view = nextTexture;
            depthStencilAttachment.view = mDepthTextureView;

            wgpu::RenderPassEncoder pass = encoder.beginRenderPass(renderPassDescriptor);
            pass.setPipeline(mPbrPipeline);

            if (!mSceneUniforms.buffer) {
                mSceneUniforms.init(mDevice);
                mSceneUniforms.value.lightColor = {1, 1, 1, 1};
                mSceneUniforms.value.lightInWorld = {0, 0, 5, 1};
            }

            int width, height;
            glfwGetFramebufferSize(mWindow.get(), &width, &height);
            float aspect = static_cast<float>(width) / static_cast<float>(height);
            mSceneUniforms.value.cameraToClip = computeCameraToClip(mFovDegrees * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
            mSceneUniforms.value.worldToCamera = mCameraInWorld.matrix().inverse().cast<float>();
            mSceneUniforms.value.cameraInWorld = mCameraInWorld.position().cast<float>().homogeneous();
            mSceneUniforms.enqueueWrite();

            wgpu::BindGroupEntry entry;
            entry.binding = 0;
            entry.buffer = mSceneUniforms.buffer;
            entry.size = sizeof(SceneUniforms);
            wgpu::BindGroupDescriptor descriptor;
            descriptor.layout = mPbrPipeline.getBindGroupLayout(1);
            descriptor.entryCount = 1;
            descriptor.entries = &entry;
            wgpu::BindGroup bindGroup = mDevice.createBindGroup(descriptor);
            pass.setBindGroup(1, bindGroup, 0, nullptr);

            if (mRenderModels) renderModels(pass);
            if (mRenderWireframeColliders) renderWireframeColliders();

            guiUpdate(pass);

            pass.end();

            bindGroup.release();
            pass.release();
        }

        nextTexture.release();

        wgpu::CommandBuffer commands = encoder.finish();
        mQueue.submit(commands);

        mSwapChain.present();

        for (Camera& camera: mCameras) {
            if (camera.thisUpdate) {
                camera.callback = camera.pointCloudBufferStaging.mapAsync(wgpu::MapMode::Read, 0, camera.pointCloudBufferStaging.getSize(), [](wgpu::BufferMapAsyncStatus const&) {});
                camera.thisUpdate = false;
            }
        }

        commands.release();
        encoder.release();
    }

} // namespace mrover
