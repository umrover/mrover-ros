#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getPrivateNodeHandle();

        mSaveTask = PeriodicTask{mPnh.param<float>("save_rate", 5)};
        mSaveHistory = boost::circular_buffer<SaveData>{static_cast<std::size_t>(mPnh.param<int>("save_history", 4096))};

        // for (std::string channel: {"/can/back_left/out", "/can/back_right/out", "/can/middle_left/out", "/can/middle_right/out", "/can/front_left/out", "/can/front_right/out"}) {
        //     mCanSubs.emplace_back(mNh.scribe<CAN>(channel, 1, [this, channel](CAN::ConstPtr const& msg) { canCallback(*msg, channel); }));
        // }
        mTwistSub = mNh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &SimulatorNodelet::twistCallback, this);
        mJointPositionsSub = mNh.subscribe<Position>("/arm_position_cmd", 1, &SimulatorNodelet::jointPositionsCallback, this);

        mPosePub = mNh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/linearized_pose", 1);

        mIkTargetPub = mNh.advertise<IK>("/arm_ik", 1);

        initWindow();

        initPhysics();

        initRender();

        parseParams();

        mRunThread = std::thread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto spinSleep(Clock::time_point const& until) -> void {
        //  See: https://blat-blatnik.github.io/computerBear/making-accurate-sleep-function/
        // Idea: sleep until 1ms before the desired time, then spin until the desired time
        std::this_thread::sleep_until(until - 1ms);
        while (Clock::now() < until);
    }

    auto SimulatorNodelet::run() -> void try {
        for (Clock::duration dt{}; ros::ok();) {
            Clock::time_point beginTime = Clock::now();

            mLoopProfiler.beginLoop();

            // Note(quintin):
            // Apple sucks and does not support polling on a non-main thread (which we are currently on)
            // Instead add to ROS's global callback queue which I think will be served by the main thread
#ifdef __APPLE__
            struct PollGlfw : ros::CallbackInterface {
                auto call() -> CallResult override {
                    glfwPollEvents();
                    return Success;
                }
            };
            ros::getGlobalCallbackQueue()->addCallback(boost::make_shared<PollGlfw>());
#else
            glfwPollEvents();
#endif
            // glfwSetInputMode(mWindow.get(), GLFW_CURSOR, mInGui ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
            mLoopProfiler.measureEvent("GLFW Events");

            IK ik;
            ik.pose.position.x = mIkTarget.x();
            ik.pose.position.y = mIkTarget.y();
            ik.pose.position.z = mIkTarget.z();
            mIkTargetPub.publish(ik);
            // if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            //     URDF const& rover = it->second;
            //
            //     for (auto const& name: {"arm_a_link"s, "arm_b_link"s, "arm_c_link"s, "arm_d_link"s, "arm_e_link"s}) {
            //         auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(rover.linkNameToIndex.at(name)).m_userPtr);
            //         motor->setMaxAppliedImpulse(0.5);
            //         if (name == "arm_b_link") {
            //             motor->setPositionTarget(-TAU * 0.125, 0.5);
            //         } else if (name == "arm_c_link") {
            //             motor->setPositionTarget(TAU * 0.4, 0.5);
            //         } else {
            //             motor->setPositionTarget(0);
            //         }
            //     }
            // }

            userControls(dt);
            mLoopProfiler.measureEvent("Controls");

            if (mEnablePhysics) physicsUpdate(dt);
            mLoopProfiler.measureEvent("Physics");

            renderUpdate();
            mLoopProfiler.measureEvent("Render");

            spinSleep(beginTime + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / mTargetUpdateRate}));
            dt = Clock::now() - beginTime;

            mLoopProfiler.measureEvent("Sleep");
        }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    SimulatorNodelet::~SimulatorNodelet() {
        mRunThread.join();

        ImGui_ImplWGPU_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

} // namespace mrover
