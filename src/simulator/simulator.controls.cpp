#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void {
        // TODO(quintin): Read these from the parameter server
        constexpr auto WHEEL_DISTANCE_INNER = Meters{0.43};
        using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;
        constexpr auto WHEEL_LINEAR_TO_ANGULAR = RadiansPerMeter{1. / .13 * 50.};
        constexpr auto MAX_MOTOR_TORQUE = 2.68;

        // constexpr auto WHEEL_DISTANCE_INNER = Meters{1};
        // constexpr auto WHEEL_LINEAR_TO_ANGULAR = compound_unit<Radians, inverse<Meters>>{1 / 0.13};

        auto forward = MetersPerSecond{twist->linear.x};
        auto turn = RadiansPerSecond{twist->angular.z} * 200 * 7; // TODO(quintin): wtf

        auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
        RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
        RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

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
                btScalar velocity = (name.contains("left"sv) ? left.get() : right.get()) / 50;
                motor->setVelocityTarget(velocity);
                motor->setMaxAppliedImpulse(MAX_MOTOR_TORQUE);
            }
        }
    }

    auto SimulatorNodelet::jointPositionsCallback(Position::ConstPtr const& positions) -> void {
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            for (auto const& combined: boost::combine(positions->names, positions->positions)) {
                int linkIndex = rover.linkNameToMeta.at(boost::get<0>(combined)).index;
                float position = boost::get<1>(combined);

                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(linkIndex).m_userPtr);
                motor->setMaxAppliedImpulse(0.5);
                motor->setPositionTarget(position, 0.05);
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
        }
    }

    auto SimulatorNodelet::freeLook(Clock::duration dt) -> void {
        float flySpeed = mFlySpeed * std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        SE3d::Tangent delta_x, delta_y, delta_z;
        delta_x << Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero();
        delta_y << Eigen::Vector3d::UnitY(), Eigen::Vector3d::Zero();
        delta_z << Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero();

        // TODO: turn this into a single rplus and lplus operation
        // overloaded rplus for movement relative to camera frame
        if (glfwGetKey(mWindow.get(), mCamForwardKey) == GLFW_PRESS) {
            mCameraInWorld += delta_x * flySpeed;
        }
        if (glfwGetKey(mWindow.get(), mCamBackwardKey) == GLFW_PRESS) {
            mCameraInWorld += delta_x * -flySpeed;
        }
        if (glfwGetKey(mWindow.get(), mCamLeftKey) == GLFW_PRESS) {
            mCameraInWorld += delta_y * flySpeed;
        }
        if (glfwGetKey(mWindow.get(), mCamRightKey) == GLFW_PRESS) {
            mCameraInWorld += delta_y * -flySpeed;
        }

        // overloaded lplus for movement relative to world frame
        if (glfwGetKey(mWindow.get(), mCamUpKey) == GLFW_PRESS) {
            mCameraInWorld = delta_z * flySpeed + mCameraInWorld;
        }
        if (glfwGetKey(mWindow.get(), mCamDownKey) == GLFW_PRESS) {
            mCameraInWorld = delta_z * -flySpeed + mCameraInWorld;
        }

        Eigen::Vector2i size;
        glfwGetWindowSize(mWindow.get(), &size.x(), &size.y());

        Eigen::Vector2d center = (size / 2).cast<double>();

        Eigen::Vector2d mouse;
        glfwGetCursorPos(mWindow.get(), &mouse.x(), &mouse.y());

        Eigen::Vector2d delta = (mouse - center) * mLookSense;

        // lplus for tilt in camera frame, rplus for pan in world frame
        SO3d::Tangent delta_Ry = Eigen::Vector3d::UnitY() * delta.y();
        SO3d::Tangent delta_Rz = Eigen::Vector3d::UnitZ() * -delta.x();
        mCameraInWorld.asSO3() = delta_Rz + mCameraInWorld.asSO3() + delta_Ry;

        centerCursor();
    }

    auto SimulatorNodelet::userControls(Clock::duration dt) -> void {
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
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
        }
        if (twist) {
            twistCallback(boost::make_shared<geometry_msgs::Twist const>(*twist));
        }
    }

} // namespace mrover
