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

    auto SimulatorNodelet::run() -> void try {

        initPhysics();

        initRender();

        parseParams();

        ros::Rate rate{mTargetFps};

        twistCallback(boost::make_shared<geometry_msgs::Twist const>());

        while (ros::ok()) {
            mLoopProfiler.beginLoop();

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
                        if (key == mTogglePhysicsKey) {
                            mEnablePhysics = !mEnablePhysics;
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
            mLoopProfiler.measureEvent("SDL Events");

            userControls(rate);
            mLoopProfiler.measureEvent("Controls");

            if (mEnablePhysics) physicsUpdate(rate);
            mLoopProfiler.measureEvent("Physics");

            renderUpdate();
            mLoopProfiler.measureEvent("Render");

            rate.sleep();
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
