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

        explicit ArmTranslator(ros::NodeHandle& nh);

        auto processVelocityCmd(Velocity::ConstPtr const& msg) -> void;

        auto processPositionCmd(Position::ConstPtr const& msg) -> void;

        auto processThrottleCmd(Throttle::ConstPtr const& msg) const -> void;

        auto processJointState(sensor_msgs::JointState::ConstPtr const& msg) -> void;

        auto updateDeOffsets(ros::TimerEvent const&) -> void;

    private:
        std::vector<std::string> const mRawArmNames{"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
        std::vector<std::string> const mArmHWNames{"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};

        std::unique_ptr<ros::Publisher> mThrottlePub;
        std::unique_ptr<ros::Publisher> mVelocityPub;
        std::unique_ptr<ros::Publisher> mPositionPub;
        std::unique_ptr<ros::Publisher> mJointDataPub;

        std::size_t const mJointDEPitchIndex = std::ranges::find(mRawArmNames, "joint_de_pitch") - mRawArmNames.begin();
        std::size_t const mJointDERollIndex = std::ranges::find(mRawArmNames, "joint_de_roll") - mRawArmNames.begin();
        std::size_t const mJointDE0Index = std::ranges::find(mArmHWNames, "joint_de_0") - mArmHWNames.begin();
        std::size_t const mJointDE1Index = std::ranges::find(mArmHWNames, "joint_de_1") - mArmHWNames.begin();
        std::size_t const mJointAIndex = std::ranges::find(mArmHWNames, "joint_a") - mArmHWNames.begin();
        std::size_t const mJointBIndex = std::ranges::find(mArmHWNames, "joint_b") - mArmHWNames.begin();
        std::size_t const mJointCIndex = std::ranges::find(mArmHWNames, "joint_c") - mArmHWNames.begin();

        ros::Timer mDeOffsetTimer;

        // RadiansPerSecond mMinRadPerSecDE0;
        // RadiansPerSecond mMinRadPerSecDE1;
        // RadiansPerSecond mMaxRadPerSecDE0;
        // RadiansPerSecond mMaxRadPerSecDE1;

        std::optional<Vector2<Radians>> mJointDePitchRoll;

        // mult
        RadiansPerMeter mJointARadiansToMeters;
        RadiansPerMeter mJointBMult;
        RadiansPerMeter mJointCMult;
        RadiansPerMeter mJointDEMult;


        Dimensionless mJointAGearRatio;
        Dimensionless mJointBGearRatio;
        Dimensionless mJointCGearRatio;
        Dimensionless mJointDEGearRatio;
    
        Dimensionless PITCH_ROLL_TO_01_SCALE;
        Matrix2<Dimensionless> PITCH_ROLL_TO_01_SCALED;
        Matrix2<Dimensionless> const PITCH_ROLL_TO_0_1 = (Matrix2<Dimensionless>{} << 1, 1, -1, 1).finished();


        // ros::Subscriber mJointDEPitchPosSub;
        // ros::Subscriber mJointDERollPosSub;

        ros::Subscriber mThrottleSub;
        ros::Subscriber mVelocitySub;
        ros::Subscriber mPositionSub;
        ros::Subscriber mJointDataSub;

        std::unordered_map<std::string, ros::ServiceClient> mAdjustClientsByArmHwNames;
    };

} // namespace mrover
