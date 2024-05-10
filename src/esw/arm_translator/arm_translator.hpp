#pragma once

#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <unordered_map>

#include <XmlRpcValue.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <mrover/AdjustMotor.h>
#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <ros/timer.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#include <Eigen/Dense>

#include <units/units.hpp>

namespace mrover {

    template<typename E>
    using Vector2 = Eigen::Matrix<E, 2, 1>;

    template<typename E>
    using Matrix2 = Eigen::Matrix<E, 2, 2>;

    class ArmTranslator {
    public:
        ArmTranslator() = default;

        ArmTranslator(ArmTranslator const&) = delete;
        ArmTranslator(ArmTranslator&&) = delete;

        auto operator=(ArmTranslator const&) -> ArmTranslator& = delete;
        auto operator=(ArmTranslator&&) -> ArmTranslator& = delete;

        explicit ArmTranslator(ros::NodeHandle& nh);

        auto processVelocityCmd(Velocity::ConstPtr const& msg) -> void;

        auto processPositionCmd(Position::ConstPtr const& msg) -> void;

        auto processThrottleCmd(Throttle::ConstPtr const& msg) const -> void;

        auto processJointState(sensor_msgs::JointState::ConstPtr const& msg) -> void;

        auto updateDeOffsets(ros::TimerEvent const&) -> void;

    private:
        std::vector<std::string> const mArmHWNames{"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};

        std::unique_ptr<ros::Publisher> mThrottlePub;
        std::unique_ptr<ros::Publisher> mVelocityPub;
        std::unique_ptr<ros::Publisher> mPositionPub;
        std::unique_ptr<ros::Publisher> mJointDataPub;

        ros::Timer mDeOffsetTimer;

        std::optional<Vector2<Radians>> mJointDePitchRoll;

        ros::Subscriber mThrottleSub;
        ros::Subscriber mVelocitySub;
        ros::Subscriber mPositionSub;
        ros::Subscriber mJointDataSub;

        std::unordered_map<std::string, ros::ServiceClient> mAdjustClientsByArmHwNames;
    };

} // namespace mrover
