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
        forEachArmMotor(message->names, message->positions, [&](btMultiBodyJointMotor* motor, float position) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(position, 0.05);
            motor->setVelocityTarget(0, 1);
        });
    }

    auto SimulatorNodelet::armVelocitiesCallback(Velocity::ConstPtr const& message) -> void {
        forEachArmMotor(message->names, message->velocities, [&](btMultiBodyJointMotor* motor, float velocity) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(velocity, 0.5);
        });
    }

    auto SimulatorNodelet::armThrottlesCallback(Throttle::ConstPtr const& message) -> void {
        forEachArmMotor(message->names, message->throttles, [&](btMultiBodyJointMotor* motor, float throttle) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(throttle, 0.5);
        });
    }

    auto SimulatorNodelet::mastPositionsCallback(Position::ConstPtr const& message) -> void {
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            assert(message->names.size() == message->positions.size());
            for (std::size_t i = 0; i < message->names.size(); ++i) {
                if (std::string const& name = message->names[i]; name == "mast_gimbal_z") {
                    float position = message->positions[i];
                    btMultibodyLink& link = rover.physics->getLink(rover.linkNameToMeta.at("zed_mini_camera").index);
                    auto* motor = std::bit_cast<btMultiBodyJointMotor*>(link.m_userPtr);
                    assert(motor);
                    motor->setMaxAppliedImpulse(0.5);
                    motor->setPositionTarget(position, 0.05);
                } else if (name == "mast_gimbal_y") {

                } else {
                    ROS_WARN_STREAM(std::format("Unknown mast joint: {}", name));
                }
            }
        }
    }

    // TODO(quintin): Remove this duplication
    auto SimulatorNodelet::mastThrottleCallback(Throttle::ConstPtr const& message) -> void {
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            assert(message->names.size() == message->throttles.size());
            for (std::size_t i = 0; i < message->names.size(); ++i) {
                if (std::string const& name = message->names[i]; name == "mast_gimbal_z") {
                    float position = message->throttles[i];
                    btMultibodyLink& link = rover.physics->getLink(rover.linkNameToMeta.at("zed_mini_camera").index);
                    auto* motor = std::bit_cast<btMultiBodyJointMotor*>(link.m_userPtr);
                    assert(motor);
                    motor->setMaxAppliedImpulse(0.5);
                    motor->setPositionTarget(position, 0.05);
                    ROS_INFO("Gimbal going to position target");
                } else if (name == "mast_gimbal_y") {

                } else {
                    ROS_WARN_STREAM(std::format("Unknown mast joint: {}", name));
                }
            }
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
            if (key == mToggleCameraLockKey) {
                if (mCameraInRoverTarget) {
                    mCameraInRoverTarget = std::nullopt;
                    
                    Eigen::Matrix3d rotationMatrix = mCameraInWorld.transform().block<3, 3>(0, 0);

                    Eigen::Vector3d left = rotationMatrix.col(1);
                    Eigen::Vector3d leftWithoutRoll = left;
                    leftWithoutRoll.z() = 0;

                    auto q = Eigen::Quaterniond{}.setFromTwoVectors(left, leftWithoutRoll);
                    mCameraInWorld.asSO3() = SO3d{q} * mCameraInWorld.asSO3();
                } else {
                    setCameraInRoverTarget();
                }
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
        // TODO: Is there a tidy way to combine this with the above?
        mCameraInWorld.asSO3() = worldRelativeAngularVelocity + mCameraInWorld.asSO3();

        centerCursor();
    }

    auto SimulatorNodelet::cameraLock([[maybe_unused]] Clock::duration dt) -> void {
        if (!mCameraInRoverTarget) setCameraInRoverTarget();

        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3d roverInWorld = rover.linkInWorld("base_link");
            SE3d cameraInWorldTarget = roverInWorld * mCameraInRoverTarget.value();
            mCameraInWorld = manif::interpolate_slerp(mCameraInWorld, cameraInWorldTarget, mCameraLockSlerp);
        }
        centerCursor();
    }

    auto SimulatorNodelet::setCameraInRoverTarget() -> void {
        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3d roverInWorld = rover.linkInWorld("base_link");
            mCameraInRoverTarget = roverInWorld.inverse() * mCameraInWorld;
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

        if (mCameraInRoverTarget)
            cameraLock(dt);
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