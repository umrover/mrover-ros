#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mRunThread = std::thread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto SimulatorNodelet::freeLook() -> void {
        if (mInGui) return;

        Uint8 const* state = SDL_GetKeyboardState(nullptr);
        if (state[mRightKey]) {
            mCameraInWorld = SE3{R3{0.0, -mFlySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (state[mLeftKey]) {
            mCameraInWorld = SE3{R3{0.0, mFlySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (state[mForwardKey]) {
            mCameraInWorld = SE3{R3{mFlySpeed, 0.0, 0.0}, SO3{}} * mCameraInWorld;
        }
        if (state[mBackwardKey]) {
            mCameraInWorld = SE3{R3{-mFlySpeed, 0.0, 0.0}, SO3{}} * mCameraInWorld;
        }
        if (state[mUpKey]) {
            mCameraInWorld = SE3{R3{0.0, 0.0, mFlySpeed}, SO3{}} * mCameraInWorld;
        }
        if (state[mDownKey]) {
            mCameraInWorld = SE3{R3{0.0, 0.0, -mFlySpeed}, SO3{}} * mCameraInWorld;
        }

        int dx{}, dy{};
        SDL_GetRelativeMouseState(&dx, &dy);

        // Ignore mouse movement if we just exited the GUI
        // Otherwise there would be a large jump in camera position
        // Note we still need to call SDL_GetRelativeMouseState to clear the relative mouse state
        if (!mInGuiChangedThisUpdated) {
            auto turnX = static_cast<double>(-dx) * 0.01;
            auto turnY = static_cast<double>(dy) * 0.01;

            R3 r3 = mCameraInWorld.position();
            SO3 so3 = SO3{turnY, Eigen::Vector3d::UnitY()} * mCameraInWorld.rotation() * SO3{turnX, Eigen::Vector3d::UnitZ()};
            mCameraInWorld = SE3{r3, so3};
        }
    }

    auto SimulatorNodelet::run() -> void try {

        initPhysics();

        initRender();

        parseParams();

        while (ros::ok()) {
            SDL_Event event;

            while (SDL_PollEvent(&event)) {
                ImGui_ImplSDL2_ProcessEvent(&event);
                switch (event.type) {
                    case SDL_QUIT:
                        ros::requestShutdown();
                        break;
                    case SDL_KEYDOWN: {
                        Sint32 key = event.key.keysym.sym;
                        if (key == mQuitKey) {
                            ros::requestShutdown();
                        }
                        if (key == mInGuiKey) {
                            mInGui = !mInGui;
                            mInGuiChangedThisUpdated = true;
                        }
                    }
                    break;
                    default:
                        break;
                }
            }

            SDL_SetRelativeMouseMode(mInGui ? SDL_FALSE : SDL_TRUE);

            freeLook();

            physicsUpdate();

            renderUpdate();

            mInGuiChangedThisUpdated = false;
        }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    SimulatorNodelet::~SimulatorNodelet() {
        mRunThread.join();
        // When you make an OpenGL context it binds to the thread that created it
        // This destructor is called from another thread, so we need to steal the context before the member destructors are run
        SDL_GL_MakeCurrent(mWindow.get(), mGlContext.get());

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
    }

} // namespace mrover
