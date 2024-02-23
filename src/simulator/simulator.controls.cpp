#include "simulator.hpp"
#include "se3.hpp"
#include <optional>

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
            if (key == mToggleCameraLockKey) {
                mCameraLock = !mCameraLock;
                if (mCameraLock)
                    setCameraInRoverTarget();
                else
                    mCameraInRoverTarget = std::nullopt;
            }
        }
    }

    auto SimulatorNodelet::freeLook(Clock::duration dt) -> void {
        auto axis = [this](int positive, int negative) -> double {
            return (glfwGetKey(mWindow.get(), positive) == GLFW_PRESS) - (glfwGetKey(mWindow.get(), negative) == GLFW_PRESS);
        };

        float flySpeed = mFlySpeed * std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        R3 keyDelta = R3{axis(mCamForwardKey, mCamBackwardKey), axis(mCamLeftKey, mCamRightKey), axis(mCamUpKey, mCamDownKey)} * flySpeed;

        Eigen::Vector2i windowSize;
        glfwGetWindowSize(mWindow.get(), &windowSize.x(), &windowSize.y());
        Eigen::Vector2d windowCenter = (windowSize / 2).cast<double>();
        Eigen::Vector2d mouse;
        glfwGetCursorPos(mWindow.get(), &mouse.x(), &mouse.y());
        Eigen::Vector2d mouseDelta = (mouse - windowCenter) * mLookSense;

        SE3d::Tangent cameraRelativeTwist;
        cameraRelativeTwist << keyDelta.x(), keyDelta.y(), 0.0, 0.0, mouseDelta.y(), 0.0;

        SE3d::Tangent worldRelativeTwist;
        worldRelativeTwist << 0.0, 0.0, keyDelta.z(), 0.0, 0.0, 0.0;

        // The plus operator is overloaded and corresponds to lplus and rplus in the micro Lie paper
        // It applies the exponential map to the tangent space element
        // The result can be acted upon a group element (in this case SE3)
        // The side of the plus operator determines word vs. camera space application
        mCameraInWorld = worldRelativeTwist + mCameraInWorld + cameraRelativeTwist;

        SO3d::Tangent worldRelativeAngularVelocity;
        worldRelativeAngularVelocity << 0.0, 0.0, -mouseDelta.x();

        // TODO: Is there a way to combine this with the above?
        mCameraInWorld.asSO3() = worldRelativeAngularVelocity + mCameraInWorld.asSO3();

        centerCursor();
    }

    auto SimulatorNodelet::cameraLock() -> void {
        if (!mCameraInRoverTarget) setCameraInRoverTarget();

        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3 baseLinkInWorld = rover.linkInWorld("base_link");
            mCameraInWorld = mCameraInRoverTarget.value() * baseLinkInWorld;
        }
        centerCursor();
    }

    auto SimulatorNodelet::setCameraInRoverTarget() -> void {
        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3 baseLinkInWorld = rover.linkInWorld("base_link");
            //AtoC = BtoC * AtoB
            mCameraInRoverTarget = baseLinkInWorld.inverse() * mCameraInWorld;
        }
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

        if (mCameraLock)
            cameraLock();
        else
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