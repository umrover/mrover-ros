#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void {
        // TODO(quintin): Read these from the parameter server
        constexpr auto WHEEL_DISTANCE_INNER = Meters{0.43};
        using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;
        constexpr auto WHEEL_LINEAR_TO_ANGULAR = RadiansPerMeter{384.615387};
        constexpr auto MAX_MOTOR_TORQUE = 2.68;

        auto forward = MetersPerSecond{twist->linear.x};
        auto turn = RadiansPerSecond{twist->angular.z} * 500;

        auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
        RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
        RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            // TODO(quintin): motor
        }
        // for (std::string const& name: {
        //              "rover#left_bogie_link_to_front_left_axle_link",
        //              "rover#left_bogie_link_to_center_left_axle_link",
        //              "rover#left_rocker_link_to_back_left_axle_link",
        //              "rover#right_bogie_link_to_front_right_axle_link",
        //              "rover#right_bogie_link_to_center_right_axle_link",
        //              "rover#right_rocker_link_to_back_right_axle_link",
        //      }) {
        //     auto it = mJointNameToSpringHinges.find(name);
        //     if (it == mJointNameToSpringHinges.end()) continue;
        //
        //     btGeneric6DofSpring2Constraint* hinge = it->second;
        //     constexpr int Y_AXIS_INDEX = 3 + 2;
        //     hinge->enableMotor(Y_AXIS_INDEX, true);
        //     hinge->setMaxMotorForce(Y_AXIS_INDEX, MAX_MOTOR_TORQUE);
        //     hinge->setTargetVelocity(Y_AXIS_INDEX, name.contains("left"sv) ? left.get() : right.get());
        // }
    }

    auto SimulatorNodelet::jointPositiionsCallback(Position::ConstPtr const& positions) -> void {
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
