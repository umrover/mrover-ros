#include "simulator.hpp"

namespace mrover {

    constexpr int SDL_OK = 0;

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
    }

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mRunThread = std::jthread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto SimulatorNodelet::renderObject(URDF const& urdf) -> void {}

    auto SimulatorNodelet::renderUpdate() -> void {
        int w{}, h{};
        SDL_GetWindowSize(mWindow.get(), &w, &h);

        glViewport(0, 0, w, h);
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

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
                    case SDL_KEYDOWN:
                        if (event.key.keysym.sym == SDLK_ESCAPE) {
                            ros::requestShutdown();
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

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "simulator");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/SimulatorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::SimulatorNodelet, nodelet::Nodelet)
#endif
