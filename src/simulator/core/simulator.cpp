#include <SDL2/SDL.h>

#include <stdexcept>
#include <format>
#include <iostream>
#include <memory>

template<typename T, auto Creater, auto Deleter>
struct SDLPointer {
    std::unique_ptr<T, decltype([](auto* p) { Deleter(p); })> mPointer;

    static auto check(T* result) -> T* {
        if (!result) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
        return result;
    }

    template<typename... Args>
    explicit SDLPointer(Args&&... args)
        : mPointer{check(Creater(std::forward<Args>(args)...))} {}

    [[nodiscard]] auto get() const noexcept -> T* {
        return mPointer.get();
    }
};

auto main() -> int try {
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

    return EXIT_SUCCESS;
} catch (std::exception const& e) {
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
}
