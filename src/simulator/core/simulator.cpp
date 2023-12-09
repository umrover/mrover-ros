#include "simulator.hpp"

namespace mrover {

    static auto check(int result) -> void {
        if (result) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
    }

    auto SimulatorNodelet::parseParams() -> void {
        XmlRpc::XmlRpcValue objects;
        mPnh.getParam("objects", objects);

        for (auto const& [name, object]: objects) {
            if (name == "urdf") mObjects.emplace_back(URDF{object});
        }
    }

    auto SimulatorNodelet::onInit() -> void {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        parseParams();

        mRunThread = std::jthread{&SimulatorNodelet::run, this};
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

    auto SimulatorNodelet::run() -> void {
        check(SDL_Init(SDL_INIT_VIDEO));

        SDL_DisplayMode displayMode;
        check(SDL_GetDesktopDisplayMode(0, &displayMode));
        assert(displayMode.w > 0 && displayMode.h > 0);

        auto w = static_cast<int>(displayMode.w * 0.8), h = static_cast<int>(displayMode.h * 0.8);
        mWindow = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{"MRover Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL};
        mGlContext = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{mWindow.get()};

        check(SDL_GL_SetSwapInterval(1));

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
