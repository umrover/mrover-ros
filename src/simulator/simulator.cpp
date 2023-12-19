#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mTwistSub = mNh.subscribe("/cmd_vel", 1, &SimulatorNodelet::twistCallback, this);

        mRunThread = std::thread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto SimulatorNodelet::freeLook() -> void {
        if (!mHasFocus || mInGui) return;

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

        auto turnX = static_cast<double>(-dx) * 0.004;
        auto turnY = static_cast<double>(dy) * 0.004;

        R3 r3 = mCameraInWorld.position();
        SO3 so3 = SO3{turnY, Eigen::Vector3d::UnitY()} * mCameraInWorld.rotation() * SO3{turnX, Eigen::Vector3d::UnitZ()};
        mCameraInWorld = SE3{r3, so3};
    }

    auto SimulatorNodelet::run() -> void try {

        initPhysics();

        initRender();

        parseParams();

        ros::Rate rate{mTargetFps};

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
                            SDL_GetRelativeMouseState(nullptr, nullptr);
                        }
                        break;
                    }
                    case SDL_WINDOWEVENT: {
                        switch (event.window.event) {
                            case SDL_WINDOWEVENT_FOCUS_GAINED:
                                mHasFocus = true;
                                SDL_GetRelativeMouseState(nullptr, nullptr);
                                break;
                            case SDL_WINDOWEVENT_FOCUS_LOST:
                                mHasFocus = false;
                                break;
                            default:
                                break;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }

            SDL_SetRelativeMouseMode(mInGui ? SDL_FALSE : SDL_TRUE);

            freeLook();

            physicsUpdate();

            renderUpdate();

            rate.sleep();
        }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto SimulatorNodelet::twistCallback(geometry_msgs::Twist::ConstPtr const& message) -> void {
        ROS_INFO_STREAM(std::format("Received twist: {}", message->linear.x));
        // std::array names{"center_left_wheel_joint", "center_right_wheel_joint", "front_left_wheel_joint", "front_right_wheel_joint", "back_left_wheel_joint", "back_right_wheel_joint"};
        // for (auto const& name: names) {
        //     mJointNameToHinges.at(name)->enableAngularMotor(true, message->linear.x, 1000);
        // }
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
