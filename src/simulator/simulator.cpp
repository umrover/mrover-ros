#include "simulator.hpp"
#include <geometry_msgs/Twist.h>

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getPrivateNodeHandle();

        mSaveTask = PeriodicTask{mPnh.param<float>("save_rate", 5)};
        mSaveHistory = boost::circular_buffer<SaveData>{static_cast<std::size_t>(mPnh.param<int>("save_history", 4096))};

        mTwistSub = mNh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &SimulatorNodelet::twistCallback, this);
        mArmPositionsSub = mNh.subscribe<Position>("/arm_position_cmd", 1, &SimulatorNodelet::armPositionsCallback, this);
        mArmVelocitiesSub = mNh.subscribe<Velocity>("/arm_velocity_cmd", 1, &SimulatorNodelet::armVelocitiesCallback, this);
        mArmThrottlesSub = mNh.subscribe<Throttle>("/arm_throttle_cmd", 1, &SimulatorNodelet::armThrottlesCallback, this);

        mGroundTruthPub = mNh.advertise<nav_msgs::Odometry>("/ground_truth", 1);
        mGpsPub = mNh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1);
        mImuPub = mNh.advertise<ImuAndMag>("/imu/data", 1);
        mVelPub = mNh.advertise<geometry_msgs::Twist>("/gps/vel", 1);

        mGpsTask = PeriodicTask{mPnh.param<float>("gps_rate", 10)};
        mImuTask = PeriodicTask{mPnh.param<float>("imu_rate", 100)};
        mMotorStatusPub = mNh.advertise<MotorsStatus>("/drive_status", 1);
        mDriveControllerStatePub = mNh.advertise<ControllerState>("/drive_controller_data", 1);
        mArmControllerStatePub = mNh.advertise<ControllerState>("/arm_controller_data", 1);

        mIkTargetPub = mNh.advertise<IK>("/arm_ik", 1);

        initWindow();

        initPhysics();

        initRender();

        parseParams();

        twistCallback(boost::make_shared<geometry_msgs::Twist>());

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

        for (int i = mDynamicsWorld->getNumConstraints() - 1; i >= 0; --i) {
            mDynamicsWorld->removeConstraint(mDynamicsWorld->getConstraint(i));
        }

        for (int i = mDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
            btCollisionObject* object = mDynamicsWorld->getCollisionObjectArray()[i];
            mDynamicsWorld->removeCollisionObject(object);
        }
    }

} // namespace mrover
