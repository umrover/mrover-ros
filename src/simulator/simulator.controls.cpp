#include "simulator.hpp"

namespace mrover {

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

        NODELET_INFO_STREAM_ONCE(std::format("left: {} right: {}", left.get(), right.get()));

        for (auto const& name: {"left_rocker_link_to_back_left_axle_link", "left_bogie_link_to_center_left_axle_link", "left_rocker_link_to_back_left_axle_link"}) {
            mJointNameToHinges.at(name)->enableAngularMotor(true, -left.get(), MAX_MOTOR_TORQUE);
        }
        for (auto const& name: {"right_rocker_link_to_back_right_axle_link", "right_bogie_link_to_center_right_axle_link", "right_rocker_link_to_back_right_axle_link"}) {
            mJointNameToHinges.at(name)->enableAngularMotor(true, -right.get(), MAX_MOTOR_TORQUE);
        }
    }

    auto SimulatorNodelet::freeLook(ros::Rate const& rate, Uint8 const* keys) -> void {
        float flySpeed = mFlySpeed * static_cast<float>(rate.expectedCycleTime().toSec());
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

    auto SimulatorNodelet::userControls(ros::Rate const& rate) -> void {
        if (!mHasFocus || mInGui) return;

        Uint8 const* keys = SDL_GetKeyboardState(nullptr);

        freeLook(rate, keys);

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
