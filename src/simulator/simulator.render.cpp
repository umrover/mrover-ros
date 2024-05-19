#define WEBGPU_CPP_IMPLEMENTATION

#include "simulator.hpp"

namespace mrover {

    static std::string const MESHES_PATH = "package://mrover/urdf/meshes/primitives";
    static std::string const CUBE_PRIMITIVE_URI = std::format("{}/cube.fbx", MESHES_PATH);
    static std::string const SPHERE_PRIMITIVE_URI = std::format("{}/sphere.fbx", MESHES_PATH);
    static std::string const CYLINDER_PRIMITIVE_URI = std::format("{}/cylinder.fbx", MESHES_PATH);

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
        std::tie(mNormalTexture, mNormalTextureView) = makeTextureAndView(width, height, NORMAL_FORMAT, wgpu::TextureUsage::RenderAttachment, wgpu::TextureAspect::All);
        std::tie(mDepthTexture, mDepthTextureView) = makeTextureAndView(width, height, DEPTH_FORMAT, wgpu::TextureUsage::RenderAttachment, wgpu::TextureAspect::DepthOnly);
    }

    auto SimulatorNodelet::makeRenderPipelines() -> void {
        wgpu::RenderPipelineDescriptor descriptor;
        descriptor.primitive.topology = wgpu::PrimitiveTopology::TriangleList;
        descriptor.primitive.cullMode = wgpu::CullMode::Back;
        descriptor.multisample.count = 1;
        descriptor.multisample.mask = 0xFFFFFFFF;

        wgpu::DepthStencilState depthStencil;
        depthStencil.depthCompare = wgpu::CompareFunction::Less;
        depthStencil.depthWriteEnabled = true;
        depthStencil.format = DEPTH_FORMAT;
        depthStencil.stencilFront.compare = wgpu::CompareFunction::Always;
        depthStencil.stencilFront.failOp = wgpu::StencilOperation::Keep;
        depthStencil.stencilFront.depthFailOp = wgpu::StencilOperation::Keep;
        depthStencil.stencilFront.passOp = wgpu::StencilOperation::Keep;
        depthStencil.stencilBack.compare = wgpu::CompareFunction::Always;
        depthStencil.stencilBack.failOp = wgpu::StencilOperation::Keep;
        depthStencil.stencilBack.depthFailOp = wgpu::StencilOperation::Keep;
        depthStencil.stencilBack.passOp = wgpu::StencilOperation::Keep;
        descriptor.depthStencil = &depthStencil;


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


        wgpu::FragmentState fragment;
        fragment.module = mShaderModule;
        fragment.entryPoint = "fs_main";

        wgpu::BlendState colorBlend;
        colorBlend.color.srcFactor = wgpu::BlendFactor::SrcAlpha;
        colorBlend.color.dstFactor = wgpu::BlendFactor::OneMinusSrcAlpha;
        colorBlend.color.operation = wgpu::BlendOperation::Add;
        colorBlend.alpha.srcFactor = wgpu::BlendFactor::Zero;
        colorBlend.alpha.dstFactor = wgpu::BlendFactor::One;
        colorBlend.alpha.operation = wgpu::BlendOperation::Add;
        wgpu::ColorTargetState colorTarget;
        colorTarget.format = COLOR_FORMAT;
        colorTarget.blend = &colorBlend;
        colorTarget.writeMask = wgpu::ColorWriteMask::All;

        wgpu::BlendState normalBlend;
        normalBlend.color.srcFactor = wgpu::BlendFactor::SrcAlpha;
        normalBlend.color.dstFactor = wgpu::BlendFactor::OneMinusSrcAlpha;
        normalBlend.color.operation = wgpu::BlendOperation::Add;
        normalBlend.alpha.srcFactor = wgpu::BlendFactor::Zero;
        normalBlend.alpha.dstFactor = wgpu::BlendFactor::One;
        normalBlend.alpha.operation = wgpu::BlendOperation::Add;
        wgpu::ColorTargetState normalTarget;
        normalTarget.format = NORMAL_FORMAT;
        normalTarget.blend = &normalBlend;
        normalTarget.writeMask = wgpu::ColorWriteMask::All;

        std::array targets{colorTarget, normalTarget};
        fragment.targetCount = targets.size();
        fragment.targets = targets.data();
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
        std::array bindGroupLayouts{meshBindGroupLayout, sceneBindGroupLayout};
        pipelineLayoutDescriptor.bindGroupLayoutCount = bindGroupLayouts.size();
        pipelineLayoutDescriptor.bindGroupLayouts = reinterpret_cast<WGPUBindGroupLayout const*>(bindGroupLayouts.data());
        descriptor.layout = mDevice.createPipelineLayout(pipelineLayoutDescriptor);

        mPbrPipeline = mDevice.createRenderPipeline(descriptor);
        if (!mPbrPipeline) throw std::runtime_error("Failed to create WGPU render pipeline");

        // TODO(quintin): This is technically not correct. As far as I can tell getting actual wireframe rendering is pretty difficult in WGPU
        descriptor.primitive.topology = wgpu::PrimitiveTopology::LineList;
        mWireframePipeline = mDevice.createRenderPipeline(descriptor);
        if (!mWireframePipeline) throw std::runtime_error("Failed to create WGPU wireframe render pipeline");
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
        mGlfwInstance.init();
        glfwSetErrorCallback([](int error, char const* description) { throw std::runtime_error(std::format("GLFW Error {}: {}", error, description)); });
        NODELET_INFO_STREAM(std::format("Initialized GLFW Version: {}.{}.{}", GLFW_VERSION_MAJOR, GLFW_VERSION_MINOR, GLFW_VERSION_REVISION));

        int x, y, w, h;
        glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &x, &y, &w, &h);
#ifdef NDEBUG
        constexpr auto WINDOW_NAME = "MRover Simulator";
#else
        constexpr auto WINDOW_NAME = "MRover Simulator (DEBUG BUILD, MAY BE SLOW)";
#endif
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
        mWindow = GlfwPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow>{w, h, WINDOW_NAME, nullptr, nullptr};
        NODELET_INFO_STREAM(std::format("Created window of size: {}x{}", w, h));

#ifndef __APPLE__
        if (cv::Mat logo = imread(std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "mrover_logo.png", cv::IMREAD_UNCHANGED);
            logo.type() == CV_8UC4) {
            cvtColor(logo, logo, cv::COLOR_BGRA2RGBA);
            GLFWimage logoImage{
                    .width = logo.cols,
                    .height = logo.rows,
                    .pixels = logo.data,
            };
            glfwSetWindowIcon(mWindow.get(), 1, &logoImage);
        } else {
            ROS_WARN_STREAM("Failed to load logo image");
        }
#endif

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
        mNormalTextureView.release();
        mNormalTexture.destroy();
        mNormalTexture.release();
        mSwapChain.release();

        makeFramebuffers(width, height);
    }

    auto SimulatorNodelet::initRender() -> void {
        {
            wgpu::InstanceDescriptor descriptor{};
            mWgpuInstance = wgpu::createInstance(descriptor);
            if (!mWgpuInstance) throw std::runtime_error("Failed to create WGPU instance");
        }
        if (!mIsHeadless) {
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

            ROS_INFO_STREAM(std::format("\tWGPU Adapter Name: {}", properties.name));
            ROS_INFO_STREAM(std::format("\tWGPU Adapter Vendor: {}", properties.vendorName));
            ROS_INFO_STREAM(std::format("\tWGPU Adapter Driver: {}", properties.driverDescription));
        }

        wgpu::SupportedLimits limits;
        mAdapter.getLimits(&limits);

        wgpu::RequiredLimits requiredLimits = wgpu::Default;
        requiredLimits.limits.maxVertexAttributes = 4;
        requiredLimits.limits.maxVertexBuffers = 8;
        requiredLimits.limits.maxBindGroups = 2;
        requiredLimits.limits.maxUniformBuffersPerShaderStage = 4;
        requiredLimits.limits.maxUniformBufferBindingSize = 1024;
        requiredLimits.limits.maxComputeWorkgroupsPerDimension = 2048;
        requiredLimits.limits.minUniformBufferOffsetAlignment = 256;
        requiredLimits.limits.minStorageBufferOffsetAlignment = 256;

        wgpu::DeviceDescriptor deviceDescriptor;
        deviceDescriptor.requiredLimits = &requiredLimits;

        mDevice = mAdapter.requestDevice(deviceDescriptor);
        if (!mDevice) throw std::runtime_error("Failed to create WGPU device");

        mErrorCallback = mDevice.setUncapturedErrorCallback([](wgpu::ErrorType type, char const* message) {
            ROS_ERROR_STREAM(std::format("WGPU Error {}: {}", static_cast<int>(type), message));
        });

        mQueue = mDevice.getQueue();
        if (!mQueue) throw std::runtime_error("Failed to get WGPU queue");

        if (!mIsHeadless) {
            int width, height;
            glfwGetFramebufferSize(mWindow.get(), &width, &height);
            makeFramebuffers(width, height);
        }

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
        makeRenderPipelines();
        {
            std::array<wgpu::BindGroupLayoutEntry, 5> bindGroupLayoutEntries{};
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
            bindGroupLayoutEntries[2].texture.sampleType = wgpu::TextureSampleType::Float;
            bindGroupLayoutEntries[2].texture.viewDimension = wgpu::TextureViewDimension::_2D;
            bindGroupLayoutEntries[3].binding = 3;
            bindGroupLayoutEntries[3].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[3].texture.sampleType = wgpu::TextureSampleType::Depth;
            bindGroupLayoutEntries[3].texture.viewDimension = wgpu::TextureViewDimension::_2D;
            bindGroupLayoutEntries[4].binding = 4;
            bindGroupLayoutEntries[4].visibility = wgpu::ShaderStage::Compute;
            bindGroupLayoutEntries[4].buffer.type = wgpu::BufferBindingType::Storage;
            bindGroupLayoutEntries[4].buffer.minBindingSize = sizeof(Point);

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

        if (!mIsHeadless) {
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGui_ImplGlfw_InitForOther(mWindow.get(), true);
            ImGui_ImplWGPU_InitInfo initInfo;
            initInfo.DepthStencilFormat = DEPTH_FORMAT;
            initInfo.RenderTargetFormat = COLOR_FORMAT;
            initInfo.Device = mDevice;
            initInfo.NumFramesInFlight = 1;
            ImGui_ImplWGPU_Init(&initInfo);

            int x, y, w, h;
            glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &x, &y, &w, &h);

            ImGuiIO& io = ImGui::GetIO();
            ImGuiStyle& style = ImGui::GetStyle();
            float scale = h > 1500 ? 2.0f : 1.0f;
            io.FontGlobalScale = scale;
            style.ScaleAllSizes(scale);
        }
    }

    auto SimulatorNodelet::renderModel(wgpu::RenderPassEncoder& pass, Model& model, Uniform<ModelUniforms>& uniforms, SIM3 const& modelToWorld, [[maybe_unused]] bool isRoverCamera) -> void {
        if (!model.areMeshesReady()) return;

        if (!uniforms.buffer) uniforms.init(mDevice);
        uniforms.value.modelToWorld = modelToWorld.matrix().cast<float>();
        // See: http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/ for why this has to be treated specially
        // TLDR: it preserves orthogonality between normal vectors and their respective surfaces with any model scaling (including non-uniform)
        uniforms.value.modelToWorldForNormals = modelToWorld.matrix().inverse().transpose().cast<float>();
        uniforms.value.modelToWorldForNormals.col(3).setZero();
        uniforms.value.modelToWorldForNormals.row(3).setZero();
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

    auto SimulatorNodelet::renderModels(wgpu::RenderPassEncoder& pass) -> void {
        for (auto& [_, urdf]: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                for (std::size_t visualIndex = 0; urdf::VisualSharedPtr const& visual: link->visual_array) {
                    if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry)) {
                        Model& model = mUriToModel.at(urdfMesh->filename);
                        URDF::LinkMeta& meta = urdf.linkNameToMeta.at(link->name);
                        SE3d linkInWorld = urdf.linkInWorld(link->name);
                        SE3d modelInLink = btTransformToSe3(urdfPoseToBtTransform(link->visual->origin));
                        SE3d modelToWorld = linkInWorld * modelInLink;
                        renderModel(pass, model, meta.visualUniforms.at(visualIndex), SIM3{modelToWorld, R3d::Ones()});
                        visualIndex++;
                    }
                }

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };

            renderLink(renderLink, urdf.model.getRoot());
        }
    }

    auto SimulatorNodelet::renderWireframeColliders(wgpu::RenderPassEncoder& pass) -> void {
        pass.setPipeline(mWireframePipeline);

        for (auto& [_, urdf]: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                URDF::LinkMeta& meta = urdf.linkNameToMeta.at(link->name);

                if (link->collision) {
                    btMultiBodyLinkCollider* collider = meta.index == -1 ? urdf.physics->getBaseCollider() : urdf.physics->getLinkCollider(meta.index);
                    assert(collider);
                    btCollisionShape* shape = collider->getCollisionShape();
                    assert(shape);
                    SE3d linkToWorld = urdf.linkInWorld(link->name);

                    if (auto* compound = dynamic_cast<btCompoundShape*>(shape)) {
                        for (int i = 0; i < compound->getNumChildShapes(); ++i) {
                            SE3d modelInLink = btTransformToSe3(urdfPoseToBtTransform(link->collision_array.at(i)->origin));
                            SE3d modelInWorld = linkToWorld * modelInLink;
                            auto* shape = compound->getChildShape(i);
                            if (auto* box = dynamic_cast<btBoxShape const*>(shape)) {
                                btVector3 extents = box->getHalfExtentsWithoutMargin() * 2;
                                SIM3 modelToWorld{modelInWorld, R3d{extents.x(), extents.y(), extents.z()}};
                                renderModel(pass, mUriToModel.at(CUBE_PRIMITIVE_URI), meta.collisionUniforms.at(i), modelToWorld);
                            } else if (auto* sphere = dynamic_cast<btSphereShape const*>(shape)) {
                                btScalar diameter = sphere->getRadius() * 2;
                                SIM3 modelToWorld{modelInWorld, R3d{diameter, diameter, diameter}};
                                renderModel(pass, mUriToModel.at(SPHERE_PRIMITIVE_URI), meta.collisionUniforms.at(i), modelToWorld);
                            } else if (auto* cylinder = dynamic_cast<btCylinderShapeZ const*>(shape)) {
                                btVector3 extents = cylinder->getHalfExtentsWithoutMargin() * 2;
                                SIM3 modelToWorld{modelInWorld, R3d{extents.x(), extents.y(), extents.z()}};
                                renderModel(pass, mUriToModel.at(CYLINDER_PRIMITIVE_URI), meta.collisionUniforms.at(i), modelToWorld);
                            } else if (auto* mesh = dynamic_cast<btBvhTriangleMeshShape const*>(shape)) {
                                SIM3 modelToWorld{modelInWorld, R3d::Ones()};
                                renderModel(pass, mUriToModel.at(mMeshToUri.at(const_cast<btBvhTriangleMeshShape*>(mesh))), meta.collisionUniforms.at(i), modelToWorld);
                            } else {
                                NODELET_WARN_STREAM_ONCE(std::format("Tried to render unsupported collision shape: {}", shape->getName()));
                            }
                        }
                    }
                }

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };

            renderLink(renderLink, urdf.model.getRoot());
        }
    }

    // template<typename T>
    // struct Pool {
    //     boost::container::static_vector<T*, 32> container;
    //
    //     auto borrowFrom() -> T* {
    //         if (container.empty()) return new T{};
    //         T* result = container.back();
    //         container.pop_back();
    //         return result;
    //     }
    //
    //     auto returnTo(T* t) -> void {
    //         container.push_back(t);
    //     }
    // };
    //
    // Pool<sensor_msgs::PointCloud2> pointCloudPool;
    // Pool<sensor_msgs::Image> imagePool;

    auto SimulatorNodelet::camerasUpdate(wgpu::CommandEncoder encoder, wgpu::RenderPassColorAttachment& colorAttachment, wgpu::RenderPassColorAttachment& normalAttachment, wgpu::RenderPassDepthStencilAttachment& depthStencilAttachment, wgpu::RenderPassDescriptor const& renderPassDescriptor) -> void {
        // TODO(quintin): Remote duplicate code
        for (StereoCamera& stereoCamera: mStereoCameras) {
            std::size_t imageSize = stereoCamera.base.resolution.x() * stereoCamera.base.resolution.y() * 4;
            if (stereoCamera.pointCloudCallback) {
                mWgpuInstance.processEvents();
                if (stereoCamera.pointCloudStagingBuffer.getMapState() == wgpu::BufferMapState::Mapped && stereoCamera.base.stagingBuffer.getMapState() == wgpu::BufferMapState::Mapped) {
                    {
                        // auto pointCloud = boost::shared_ptr<sensor_msgs::PointCloud2>{pointCloudPool.borrowFrom(), [](sensor_msgs::PointCloud2* msg) { pointCloudPool.returnTo(msg); }};
                        auto pointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
                        pointCloud->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
                        pointCloud->is_dense = true;
                        pointCloud->width = stereoCamera.base.resolution.x();
                        pointCloud->height = stereoCamera.base.resolution.y();
                        pointCloud->header.stamp = ros::Time::now();
                        pointCloud->header.frame_id = stereoCamera.base.frameId;
                        fillPointCloudMessageHeader(pointCloud);

                        auto* fromCompute = stereoCamera.pointCloudStagingBuffer.getConstMappedRange(0, stereoCamera.pointCloudStagingBuffer.getSize());
                        auto* toMessage = pointCloud->data.data();
                        std::memcpy(toMessage, fromCompute, stereoCamera.pointCloudStagingBuffer.getSize());
                        stereoCamera.pointCloudStagingBuffer.unmap();
                        stereoCamera.pointCloudCallback = nullptr;

                        stereoCamera.pcPub.publish(pointCloud);
                    }
                    {
                        // auto image = boost::shared_ptr<sensor_msgs::Image>{imagePool.borrowFrom(), [](sensor_msgs::Image* msg) { imagePool.returnTo(msg); }};
                        auto image = boost::make_shared<sensor_msgs::Image>();
                        image->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
                        image->encoding = sensor_msgs::image_encodings::BGRA8;
                        image->width = stereoCamera.base.resolution.x();
                        image->height = stereoCamera.base.resolution.y();
                        image->step = stereoCamera.base.resolution.x() * 4;
                        image->header.stamp = ros::Time::now();
                        image->header.frame_id = stereoCamera.base.frameId;
                        image->data.resize(imageSize);

                        auto* fromRender = stereoCamera.base.stagingBuffer.getConstMappedRange(0, stereoCamera.base.stagingBuffer.getSize());
                        auto* toMessage = image->data.data();
                        std::memcpy(toMessage, fromRender, stereoCamera.base.stagingBuffer.getSize());
                        stereoCamera.base.stagingBuffer.unmap();
                        stereoCamera.base.callback = nullptr;

                        stereoCamera.base.pub.publish(image);
                    }
                }
            }
            if (!stereoCamera.pointCloudCallback && !stereoCamera.base.callback && stereoCamera.base.updateTask.shouldUpdate()) {
                {
                    colorAttachment.view = stereoCamera.base.colorTextureView;
                    normalAttachment.view = stereoCamera.base.normalTextureView;
                    depthStencilAttachment.view = stereoCamera.base.depthTextureView;

                    renderCamera(stereoCamera.base, encoder, renderPassDescriptor);
                    computeStereoCamera(stereoCamera, encoder);

                    stereoCamera.base.needsMap = true;
                }
                {
                    colorAttachment.view = stereoCamera.base.colorTextureView;
                    normalAttachment.view = stereoCamera.base.normalTextureView;
                    depthStencilAttachment.view = stereoCamera.base.depthTextureView;

                    renderCamera(stereoCamera.base, encoder, renderPassDescriptor);

                    if (!stereoCamera.base.stagingBuffer) {
                        wgpu::BufferDescriptor descriptor;
                        descriptor.usage = wgpu::BufferUsage::CopyDst | wgpu::BufferUsage::MapRead;
                        descriptor.size = imageSize;
                        stereoCamera.base.stagingBuffer = mDevice.createBuffer(descriptor);
                    }

                    wgpu::ImageCopyTexture copyTexture;
                    copyTexture.texture = stereoCamera.base.colorTexture;
                    copyTexture.aspect = wgpu::TextureAspect::All;
                    wgpu::ImageCopyBuffer copyBuffer;
                    copyBuffer.buffer = stereoCamera.base.stagingBuffer;
                    copyBuffer.layout.bytesPerRow = stereoCamera.base.resolution.x() * 4;
                    copyBuffer.layout.rowsPerImage = stereoCamera.base.resolution.y();
                    wgpu::Extent3D extent{
                            static_cast<std::uint32_t>(stereoCamera.base.resolution.x()),
                            static_cast<std::uint32_t>(stereoCamera.base.resolution.y()),
                            1,
                    };
                    encoder.copyTextureToBuffer(copyTexture, copyBuffer, extent);

                    stereoCamera.base.needsMap = true;
                }
            }
        }
        for (Camera& camera: mCameras) {
            std::size_t area = camera.resolution.x() * camera.resolution.y();
            if (camera.callback) {
                mWgpuInstance.processEvents();
                if (camera.stagingBuffer.getMapState() == wgpu::BufferMapState::Mapped) {
                    // auto image = boost::shared_ptr<sensor_msgs::Image>{imagePool.borrowFrom(), [](sensor_msgs::Image* msg) { imagePool.returnTo(msg); }};
                    auto image = boost::make_shared<sensor_msgs::Image>();
                    image->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
                    image->encoding = sensor_msgs::image_encodings::BGRA8;
                    image->width = camera.resolution.x();
                    image->height = camera.resolution.y();
                    image->step = camera.resolution.x() * 4;
                    image->header.stamp = ros::Time::now();
                    image->header.frame_id = camera.frameId;
                    image->data.resize(area * 4);

                    // Convert from BGRA to BGR
                    auto* fromRender = camera.stagingBuffer.getConstMappedRange(0, camera.stagingBuffer.getSize());
                    auto* toMessage = image->data.data();
                    std::memcpy(toMessage, fromRender, camera.stagingBuffer.getSize());
                    camera.stagingBuffer.unmap();
                    camera.callback = nullptr;

                    camera.pub.publish(image);
                }
            }
            if (!camera.callback && camera.updateTask.shouldUpdate()) {
                colorAttachment.view = camera.colorTextureView;
                normalAttachment.view = camera.normalTextureView;
                depthStencilAttachment.view = camera.depthTextureView;

                renderCamera(camera, encoder, renderPassDescriptor);

                if (!camera.stagingBuffer) {
                    wgpu::BufferDescriptor descriptor;
                    descriptor.usage = wgpu::BufferUsage::CopyDst | wgpu::BufferUsage::MapRead;
                    descriptor.size = area * 4;
                    camera.stagingBuffer = mDevice.createBuffer(descriptor);
                }

                wgpu::ImageCopyTexture copyTexture;
                copyTexture.texture = camera.colorTexture;
                copyTexture.aspect = wgpu::TextureAspect::All;
                wgpu::ImageCopyBuffer copyBuffer;
                copyBuffer.buffer = camera.stagingBuffer;
                copyBuffer.layout.bytesPerRow = camera.resolution.x() * 4;
                copyBuffer.layout.rowsPerImage = camera.resolution.y();
                wgpu::Extent3D extent{
                        static_cast<std::uint32_t>(camera.resolution.x()),
                        static_cast<std::uint32_t>(camera.resolution.y()),
                        1,
                };
                encoder.copyTextureToBuffer(copyTexture, copyBuffer, extent);

                camera.needsMap = true;
            }
        }
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        std::array<wgpu::RenderPassColorAttachment, 2> colorAttachments{};
        auto& [colorAttachment, normalAttachment] = colorAttachments;
        colorAttachment.loadOp = wgpu::LoadOp::Clear;
        colorAttachment.storeOp = wgpu::StoreOp::Store;
        colorAttachment.clearValue = {mSkyColor.x(), mSkyColor.y(), mSkyColor.z(), mSkyColor.w()};
        colorAttachment.depthSlice = wgpu::kDepthSliceUndefined;
        normalAttachment.loadOp = wgpu::LoadOp::Clear;
        normalAttachment.storeOp = wgpu::StoreOp::Store;
        normalAttachment.clearValue = {0, 0, 0, 0};
        normalAttachment.depthSlice = wgpu::kDepthSliceUndefined;

        wgpu::RenderPassDepthStencilAttachment depthStencilAttachment;
        depthStencilAttachment.depthClearValue = 1.0f;
        depthStencilAttachment.depthLoadOp = wgpu::LoadOp::Clear;
        depthStencilAttachment.depthStoreOp = wgpu::StoreOp::Store;
        depthStencilAttachment.stencilLoadOp = wgpu::LoadOp::Undefined;
        depthStencilAttachment.stencilStoreOp = wgpu::StoreOp::Undefined;
        depthStencilAttachment.stencilReadOnly = true;

        wgpu::RenderPassDescriptor renderPassDescriptor;
        renderPassDescriptor.colorAttachmentCount = colorAttachments.size();
        renderPassDescriptor.colorAttachments = colorAttachments.data();
        renderPassDescriptor.depthStencilAttachment = &depthStencilAttachment;

        wgpu::CommandEncoder encoder = mDevice.createCommandEncoder();

        camerasUpdate(encoder, colorAttachment, normalAttachment, depthStencilAttachment, renderPassDescriptor);

        if (!mIsHeadless) {
            wgpu::TextureView nextTexture = mSwapChain.getCurrentTextureView();
            if (!nextTexture) throw std::runtime_error("Failed to get WGPU next texture view");

            colorAttachment.view = nextTexture;
            normalAttachment.view = mNormalTextureView;
            depthStencilAttachment.view = mDepthTextureView;

            wgpu::RenderPassEncoder pass = encoder.beginRenderPass(renderPassDescriptor);
            pass.setPipeline(mPbrPipeline);

            if (!mSceneUniforms.buffer) {
                mSceneUniforms.init(mDevice);
                mSceneUniforms.value.lightColor = {1, 1, 1, 1};
                mSceneUniforms.value.lightInWorld = {0, 0, 20, 1};
            }

            int width, height;
            glfwGetFramebufferSize(mWindow.get(), &width, &height);
            float aspect = static_cast<float>(width) / static_cast<float>(height);
            mSceneUniforms.value.cameraToClip = computeCameraToClip(mFovDegrees * DEG_TO_RAD, aspect, NEAR, FAR).cast<float>();
            mSceneUniforms.value.worldToCamera = mCameraInWorld.inverse().transform().cast<float>();
            mSceneUniforms.value.cameraInWorld = mCameraInWorld.translation().cast<float>().homogeneous();
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
            if (mRenderWireframeColliders) renderWireframeColliders(pass);

            guiUpdate(pass);

            pass.end();
            pass.release();

            bindGroup.release();

            nextTexture.release();
        }

        wgpu::CommandBuffer commands = encoder.finish();
        mQueue.submit(commands);

#ifdef __APPLE__
        {
            // Temporary fix...
            // See: https://issues.chromium.org/issues/338710345
            // This only happens on M2/M3 (M1 is fine)
            bool isWorkDone = false;
            auto workDoneCallback = mQueue.onSubmittedWorkDone([&isWorkDone](wgpu::QueueWorkDoneStatus const& status) {
                if (status == wgpu::QueueWorkDoneStatus::Success) {
                    isWorkDone = true;
                }
            });
            while (!isWorkDone) {
                mDevice.tick();
            }
        }
#endif

        if (!mIsHeadless) mSwapChain.present();

        // TODO(quintin): Remote duplicate code
        for (StereoCamera& stereoCamera: mStereoCameras) {
            if (stereoCamera.base.needsMap) {
                stereoCamera.pointCloudCallback = stereoCamera.pointCloudStagingBuffer.mapAsync(wgpu::MapMode::Read, 0, stereoCamera.pointCloudStagingBuffer.getSize(), [](wgpu::BufferMapAsyncStatus const&) {});

                stereoCamera.base.callback = stereoCamera.base.stagingBuffer.mapAsync(wgpu::MapMode::Read, 0, stereoCamera.base.stagingBuffer.getSize(), [](wgpu::BufferMapAsyncStatus const&) {});

                stereoCamera.base.needsMap = false;
            }
        }
        for (Camera& camera: mCameras) {
            if (camera.needsMap) {
                camera.callback = camera.stagingBuffer.mapAsync(wgpu::MapMode::Read, 0, camera.stagingBuffer.getSize(), [](wgpu::BufferMapAsyncStatus const&) {});
                camera.needsMap = false;
            }
        }

        commands.release();
        encoder.release();
    }

} // namespace mrover