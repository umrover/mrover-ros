#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mThread = std::jthread{&SimulatorNodelet::update, this};
    }

    auto SimulatorNodelet::update() -> void {
        auto window = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{"MRover Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, SDL_WINDOW_OPENGL};

        auto context = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{window.get()};

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
        }
    }
}

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
