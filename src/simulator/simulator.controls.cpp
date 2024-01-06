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
        auto turn = RadiansPerSecond{twist->angular.z} * 200 * 8;

        auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
        RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
        RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            for (std::string const& name: {
                         "front_left_axle_link",
                         "center_left_axle_link",
                         "back_left_axle_link",
                         "front_right_axle_link",
                         "center_right_axle_link",
                         "back_right_axle_link",
                 }) {
                int axle = rover.linkNameToIndex.at(name);
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(axle).m_userPtr);
                btScalar velocity = (name.contains("left"sv) ? left.get() : right.get()) / 50;
                motor->setVelocityTarget(velocity);
                motor->setMaxAppliedImpulse(MAX_MOTOR_TORQUE);
            }

            for (std::string const& name: {
                         "front_left_wheel_link",
                         "center_left_wheel_link",
                         "back_left_wheel_link",
                         "front_right_wheel_link",
                         "center_right_wheel_link",
                         "back_right_wheel_link",
                 }) {
                auto wheel = rover.linkNameToIndex.at(name);
                auto axle = rover.physics->getParent(wheel);
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(wheel).m_userPtr);
                btScalar velocity = rover.physics->getJointVel(axle);
                motor->setVelocityTarget(velocity);
                motor->setMaxAppliedImpulse(MAX_MOTOR_TORQUE);
            }
        }
    }

    auto SimulatorNodelet::jointPositionsCallback(Position::ConstPtr const& positions) -> void {
        // TODO(quintin): motor
        // for (auto const& combined: boost::combine(positions->names, positions->positions)) {
        //     std::string name = std::format("rover#{}", combined.get<0>());
        //
        //     auto it = mJointNameToHinges.find(name);
        //     if (it == mJointNameToHinges.end()) continue;
        //
        //     btHingeConstraint* hinge = it->second;
        //     hinge->enableMotor(true);
        //     hinge->setMaxMotorImpulse(1.0);
        //     hinge->setMotorTarget(combined.get<1>(), 1.0f / ImGui::GetIO().Framerate);
        // }
    }

    auto SimulatorNodelet::freeLook(Clock::duration dt, Uint8 const* keys) -> void {
        float flySpeed = mFlySpeed * std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        if (keys[mCamRightKey]) {
            mCameraInWorld = SE3{R3{0.0, -flySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (keys[mCamLeftKey]) {
            mCameraInWorld = SE3{R3{0.0, flySpeed, 0}, SO3{}} * mCameraInWorld;
        }
        if (keys[mCamForwardKey]) {
            mCameraInWorld = SE3{R3{flySpeed, 0.0, 0.0}, SO3{}} * mCameraInWorld;
        }
        if (keys[mCamBackwardKey]) {
            mCameraInWorld = SE3{R3{-flySpeed, 0.0, 0.0}, SO3{}} * mCameraInWorld;
        }
        if (keys[mCamUpKey]) {
            mCameraInWorld = mCameraInWorld * SE3{R3{0.0, 0.0, flySpeed}, SO3{}};
        }
        if (keys[mCamDownKey]) {
            mCameraInWorld = mCameraInWorld * SE3{R3{0.0, 0.0, -flySpeed}, SO3{}};
        }

        int dx{}, dy{};
        SDL_GetRelativeMouseState(&dx, &dy);

        auto turnX = static_cast<double>(-dx) * mLookSense;
        auto turnY = static_cast<double>(dy) * mLookSense;

        R3 p = mCameraInWorld.position();
        SO3 q = SO3{turnY, Eigen::Vector3d::UnitY()} * mCameraInWorld.rotation() * SO3{turnX, Eigen::Vector3d::UnitZ()};
        mCameraInWorld = SE3{p, q};
    }

    auto SimulatorNodelet::userControls(Clock::duration dt) -> void {
        if (!mHasFocus || mInGui) return;

        Uint8 const* keys = SDL_GetKeyboardState(nullptr);

        freeLook(dt, keys);

        std::optional<geometry_msgs::Twist> twist;
        if (keys[mRoverRightKey]) {
            if (!twist) twist.emplace();
            twist->angular.z = -mRoverAngularSpeed;
        }
        if (keys[mRoverLeftKey]) {
            if (!twist) twist.emplace();
            twist->angular.z = mRoverAngularSpeed;
        }
        if (keys[mRoverForwardKey]) {
            if (!twist) twist.emplace();
            twist->linear.x = mRoverLinearSpeed;
        }
        if (keys[mRoverBackwardKey]) {
            if (!twist) twist.emplace();
            twist->linear.x = -mRoverLinearSpeed;
        }
        if (keys[mRoverStopKey]) {
            if (!twist) twist.emplace();
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
        }
        if (twist) {
            twistCallback(boost::make_shared<geometry_msgs::Twist const>(*twist));
        }
    }

} // namespace mrover
