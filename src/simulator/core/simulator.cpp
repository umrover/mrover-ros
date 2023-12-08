#include "../pch.hpp"

namespace mrover {
    template<typename T, auto Creater, auto Deleter>
    class SDLPointer {
        std::unique_ptr<T, decltype([](auto* p) { Deleter(p); })> mPointer;

        static auto check(T* result) -> T* {
            if (!result) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
            return result;
        }

    public:
        template<typename... Args>
        explicit SDLPointer(Args&&... args)
            : mPointer{check(Creater(std::forward<Args>(args)...))} {}

        [[nodiscard]] auto get() const noexcept -> T* {
            return mPointer.get();
        }
    };

    class SimulatorNodelet : public nodelet::Nodelet {
        std::jthread mThread;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override = default;

        void onInit() override {
            mThread = std::jthread{&SimulatorNodelet::update, this};

        }

    private:
        auto update() -> void {
            auto window = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{"MRover Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, SDL_WINDOW_OPENGL};

            auto context = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{window.get()};

            bool running = true;
            while (running) {
                SDL_Event event;

                while (SDL_PollEvent(&event)) {
                    switch (event.type) {
                        case SDL_KEYDOWN:
                            if (event.key.keysym.sym == SDLK_ESCAPE) {
                                running = false;
                            }
                            break;
                        case SDL_QUIT:
                            running = false;
                            break;
                        default:
                            break;
                    }
                }
            }

            ros::requestShutdown();
        }
    };
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
