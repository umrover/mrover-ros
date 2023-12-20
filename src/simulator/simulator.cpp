#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        // for (std::string channel: {"/can/back_left/out", "/can/back_right/out", "/can/middle_left/out", "/can/middle_right/out", "/can/front_left/out", "/can/front_right/out"}) {
        //     mCanSubs.emplace_back(mNh.subscribe<CAN>(channel, 1, [this, channel](CAN::ConstPtr const& msg) { canCallback(*msg, channel); }));
        // }
        mTwistSub = mNh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &SimulatorNodelet::twistCallback, this);

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
            mCameraInWorld = mCameraInWorld * SE3{R3{0.0, 0.0, mFlySpeed}, SO3{}};
        }
        if (state[mDownKey]) {
            mCameraInWorld = mCameraInWorld * SE3{R3{0.0, 0.0, -mFlySpeed}, SO3{}};
        }

        int dx{}, dy{};
        SDL_GetRelativeMouseState(&dx, &dy);

        auto turnX = static_cast<double>(-dx) * mLookSense;
        auto turnY = static_cast<double>(dy) * mLookSense;

        R3 p = mCameraInWorld.position();
        SO3 q = SO3{turnY, Eigen::Vector3d::UnitY()} * mCameraInWorld.rotation() * SO3{turnX, Eigen::Vector3d::UnitZ()};
        mCameraInWorld = SE3{p, q};
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

            physicsUpdate(rate);

            renderUpdate();

            rate.sleep();
        }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto SimulatorNodelet::twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void {
        // TODO: read these from the parameter server
        constexpr auto WHEEL_DISTANCE_INNER = Meters{0.43};
        using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;
        constexpr auto WHEEL_LINEAR_TO_ANGULAR = RadiansPerMeter{384.615387};
        constexpr auto MAX_MOTOR_TORQUE = 2.68;

        auto forward = MetersPerSecond{twist->linear.x};
        auto turn = RadiansPerSecond{twist->angular.z} * 50;

        auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
        RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
        RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

        for (auto const& name: {"center_left_axle_joint", "front_left_axle_joint", "back_left_axle_joint"}) {
            mJointNameToHinges.at(name)->enableAngularMotor(true, left.get(), MAX_MOTOR_TORQUE);
        }
        for (auto const& name: {"center_right_axle_joint", "front_right_axle_joint", "back_right_axle_joint"}) {
            mJointNameToHinges.at(name)->enableAngularMotor(true, right.get(), MAX_MOTOR_TORQUE);
        }
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
