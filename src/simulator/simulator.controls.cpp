#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void {
        // TODO(quintin): Read these from the parameter server
        constexpr auto WHEEL_DISTANCE_INNER = Meters{0.43};
        using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;
        constexpr auto WHEEL_LINEAR_TO_ANGULAR = RadiansPerMeter{1. / .13};
        constexpr auto MAX_MOTOR_TORQUE = 2.68;

        auto forward = MetersPerSecond{twist->linear.x};
        auto turn = RadiansPerSecond{twist->angular.z} * 3; // TODO(quintin): Remove this hack

        auto delta = turn / Radians{1} * WHEEL_DISTANCE_INNER;

        RadiansPerSecond left = (forward - delta) * WHEEL_LINEAR_TO_ANGULAR;
        RadiansPerSecond right = (forward + delta) * WHEEL_LINEAR_TO_ANGULAR;

        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            for (std::string const& name: {
                         "front_left_wheel_link",
                         "center_left_wheel_link",
                         "back_left_wheel_link",
                         "front_right_wheel_link",
                         "center_right_wheel_link",
                         "back_right_wheel_link",
                 }) {
                int wheelIndex = rover.linkNameToMeta.at(name).index;
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(wheelIndex).m_userPtr);
                btScalar velocity = name.contains("left"sv) ? left.get() : right.get();
                motor->setVelocityTarget(velocity);
                motor->setMaxAppliedImpulse(MAX_MOTOR_TORQUE);
            }
        }
    }

    auto SimulatorNodelet::armPositionsCallback(Position::ConstPtr const& message) -> void {
        forEachWithMotor(message->names, message->positions, [&](btMultiBodyJointMotor* motor, float position) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(position, 0.05);
            motor->setVelocityTarget(0, 1);
        });
    }

    auto SimulatorNodelet::armVelocitiesCallback(Velocity::ConstPtr const& message) -> void {
        forEachWithMotor(message->names, message->velocities, [&](btMultiBodyJointMotor* motor, float velocity) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(velocity, 0.5);
        });
    }

    auto SimulatorNodelet::armThrottlesCallback(Throttle::ConstPtr const& message) -> void {
        forEachWithMotor(message->names, message->throttles, [&](btMultiBodyJointMotor* motor, float throttle) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(throttle, 0.5);
        });
    }

    auto SimulatorNodelet::mastPositionsCallback(Position::ConstPtr const& message) -> void {
        assert(message->names == std::vector<std::string>{"mast_joint"});
        assert(message->positions.size() == 1);
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            btMultibodyLink& link = rover.physics->getLink(rover.linkNameToMeta.at("zed_mini_camera").index);
            auto* motor = std::bit_cast<btMultiBodyJointMotor*>(link.m_userPtr);
            assert(motor);
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(message->positions.front(), 0.05);
        }
    }

    auto SimulatorNodelet::centerCursor() -> void {
        Eigen::Vector2i size;
        glfwGetWindowSize(mWindow.get(), &size.x(), &size.y());
        Eigen::Vector2i center = size / 2;
        glfwSetCursorPos(mWindow.get(), center.x(), center.y());
    }

    auto SimulatorNodelet::keyCallback(int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) -> void {
        if (action == GLFW_PRESS) {
            if (key == mQuitKey) ros::requestShutdown();
            if (key == mTogglePhysicsKey) mEnablePhysics = !mEnablePhysics;
            if (key == mToggleRenderModelsKey) mRenderModels = !mRenderModels;
            if (key == mToggleRenderWireframeCollidersKey) mRenderWireframeColliders = !mRenderWireframeColliders;
            if (key == mInGuiKey) {
                mInGui = !mInGui;
                if (!mInGui) centerCursor();
            }
        }
    }

    auto SimulatorNodelet::freeLook(Clock::duration dt) -> void {
        float flySpeed = mFlySpeed * std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        if (glfwGetKey(mWindow.get(), mCamRightKey) == GLFW_PRESS) {
            mCameraInWorld = SE3{R3{0, -flySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (glfwGetKey(mWindow.get(), mCamLeftKey) == GLFW_PRESS) {
            mCameraInWorld = SE3{R3{0, flySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (glfwGetKey(mWindow.get(), mCamForwardKey) == GLFW_PRESS) {
            mCameraInWorld = SE3{R3{flySpeed, 0, 0}, SO3{}} * mCameraInWorld;
        }
        if (glfwGetKey(mWindow.get(), mCamBackwardKey) == GLFW_PRESS) {
            mCameraInWorld = SE3{R3{-flySpeed, 0, 0}, SO3{}} * mCameraInWorld;
        }
        if (glfwGetKey(mWindow.get(), mCamUpKey) == GLFW_PRESS) {
            mCameraInWorld = mCameraInWorld * SE3{R3{0, 0, flySpeed}, SO3{}};
        }
        if (glfwGetKey(mWindow.get(), mCamDownKey) == GLFW_PRESS) {
            mCameraInWorld = mCameraInWorld * SE3{R3{0, 0, -flySpeed}, SO3{}};
        }

        Eigen::Vector2i size;
        glfwGetWindowSize(mWindow.get(), &size.x(), &size.y());

        Eigen::Vector2d center = (size / 2).cast<double>();

        Eigen::Vector2d mouse;
        glfwGetCursorPos(mWindow.get(), &mouse.x(), &mouse.y());

        Eigen::Vector2d delta = (mouse - center) * mLookSense;

        // TODO(quintin): use lie algebra more here? we have a perturbation in the tangent space
        R3 p = mCameraInWorld.position();
        SO3 q = SO3{delta.y(), R3::UnitY()} * mCameraInWorld.rotation() * SO3{-delta.x(), R3::UnitZ()};
        mCameraInWorld = SE3{p, q};

        centerCursor();
    }

    auto SimulatorNodelet::userControls(Clock::duration dt) -> void {
        if (mPublishIk) {
            IK ik;
            ik.pose.position.x = mIkTarget.x();
            ik.pose.position.y = mIkTarget.y();
            ik.pose.position.z = mIkTarget.z();
            mIkTargetPub.publish(ik);
        }

        if (!mHasFocus || mInGui) return;

        freeLook(dt);

        std::optional<geometry_msgs::Twist> twist;
        if (glfwGetKey(mWindow.get(), mRoverRightKey) == GLFW_PRESS) {
            if (!twist) twist.emplace();
            twist->angular.z = -mRoverAngularSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverLeftKey) == GLFW_PRESS) {
            if (!twist) twist.emplace();
            twist->angular.z = mRoverAngularSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverForwardKey) == GLFW_PRESS) {
            if (!twist) twist.emplace();
            twist->linear.x = mRoverLinearSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverBackwardKey) == GLFW_PRESS) {
            if (!twist) twist.emplace();
            twist->linear.x = -mRoverLinearSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverStopKey) == GLFW_PRESS) {
            if (!twist) twist.emplace();
            twist->linear.x = 0;
            twist->angular.z = 0;
        }
        if (twist) {
            twistCallback(boost::make_shared<geometry_msgs::Twist const>(*twist));
        }
    }

} // namespace mrover
