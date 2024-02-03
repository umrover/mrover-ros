#pragma once

#include <format>
#include <memory>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include "units/units.hpp"

#include "joint_de_translation.hpp"
#include "matrix_helper.hpp"
#include "read_from_ros_param.hpp"
#include <mrover/AdjustMotor.h>
#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>


namespace mrover {

    class ArmTranslator {
    public:
        ArmTranslator() = default;

        ArmTranslator(ros::NodeHandle& nh);

        void processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg);

        void processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg);

        void processVelocityCmd(Velocity::ConstPtr const& msg);

        void processPositionCmd(Position::ConstPtr const& msg);

        void processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg);

        bool adjustServiceCallback(AdjustMotor::Request& req, AdjustMotor::Response& res);

        bool calibrateServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    private:
        void processThrottleCmd(Throttle::ConstPtr const& msg);

        static void clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2);
        static void mapValue(float& val, float inputMinValue, float inputMaxValue, float outputMinValue, float outputMaxValue);

        bool jointDEIsCalibrated();

        void updatePositionOffsets();

        const std::vector<std::string> mRawArmNames = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
        const std::vector<std::string> mArmHWNames = {"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};
        std::unique_ptr<ros::Publisher> mThrottlePub;
        std::unique_ptr<ros::Publisher> mVelocityPub;
        std::unique_ptr<ros::Publisher> mPositionPub;
        std::unique_ptr<ros::Publisher> mJointDataPub;
        const size_t mJointDEPitchIndex = std::find(mRawArmNames.begin(), mRawArmNames.end(), "joint_de_pitch") - mRawArmNames.begin();
        const size_t mJointDERollIndex = std::find(mRawArmNames.begin(), mRawArmNames.end(), "joint_de_roll") - mRawArmNames.begin();
        const size_t mJointDE0Index = std::find(mArmHWNames.begin(), mArmHWNames.end(), "joint_de_0") - mArmHWNames.begin();
        const size_t mJointDE1Index = std::find(mArmHWNames.begin(), mArmHWNames.end(), "joint_de_1") - mArmHWNames.begin();

        std::optional<Radians> mJointDE0PosOffset = Radians{0};
        std::optional<Radians> mJointDE1PosOffset = Radians{0};

        Radians mJointDEPitchOffset;
        Radians mJointDERollOffset;

        std::optional<Radians> mCurrentRawJointDEPitch;
        std::optional<Radians> mCurrentRawJointDERoll;
        std::optional<Radians> mCurrentRawJointDE0Position;
        std::optional<Radians> mCurrentRawJointDE1Position;

        RadiansPerSecond mMinRadPerSecDE0{};
        RadiansPerSecond mMinRadPerSecDE1{};
        RadiansPerSecond mMaxRadPerSecDE0{};
        RadiansPerSecond mMaxRadPerSecDE1{};

        ros::Subscriber mJointDEPitchPosSub;
        ros::Subscriber mJointDERollPosSub;

        ros::Subscriber mThrottleSub;
        ros::Subscriber mVelocitySub;
        ros::Subscriber mPositionSub;
        ros::Subscriber mArmHWJointDataSub;

        // TODO:(owen) unique_ptr servers? unique_ptr clients? Both? Neither? The world may never know. (try to learn)
        std::unordered_map<std::string, std::unique_ptr<ros::ServiceServer>> mAdjustServersByRawArmNames;
        // std::unordered_map<std::string, std::unique_ptr<ros::ServiceServer> > mCalibrateServer;

        std::unordered_map<std::string, ros::ServiceClient> mAdjustClientsByArmHWNames;
        // std::unique_ptr<ros::ServiceClient> mCalibrateClient;

        std::optional<float> mJointDEPitchAdjust;
        std::optional<float> mJointDERollAdjust;
    };

} // namespace mrover