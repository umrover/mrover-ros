#pragma once

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include "units/units.hpp"

#include "joint_de_translation.hpp"
#include "matrix_helper.hpp"
#include "read_from_ros_param.hpp"
#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

namespace mrover {

    class JointDE {
    public:
        JointDE() = default;

        JointDE(ros::NodeHandle& nh);

        void processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg);

        void processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg);

        void processVelocityCmd(Velocity::ConstPtr const& msg);

        void processPositionCmd(Position::ConstPtr const& msg);

        void processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg);

    private:
        void processThrottleCmd(Throttle::ConstPtr const& msg);

        static void clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2);

        bool jointDEIsCalibrated();

        void updatePositionOffsets();

        const std::vector<std::string> rawArmNames = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
        const std::vector<std::string> armHWNames = {"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};
        std::unique_ptr<ros::Publisher> throttlePub;
        std::unique_ptr<ros::Publisher> velocityPub;
        std::unique_ptr<ros::Publisher> positionPub;
        std::unique_ptr<ros::Publisher> jointDataPub;
        const size_t joint_de_pitch_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_pitch") - rawArmNames.begin();
        const size_t joint_de_roll_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_roll") - rawArmNames.begin();
        const size_t joint_de_0_index = std::find(armHWNames.begin(), armHWNames.end(), "joint_de_0") - armHWNames.begin();
        const size_t joint_de_1_index = std::find(armHWNames.begin(), armHWNames.end(), "joint_de_1") - armHWNames.begin();

        std::optional<Radians> jointDE0PosOffset = Radians{0};
        std::optional<Radians> jointDE1PosOffset = Radians{0};

        std::optional<Radians> currentRawJointDEPitch;
        std::optional<Radians> currentRawJointDERoll;
        std::optional<Radians> currentRawJointDE0Position;
        std::optional<Radians> currentRawJointDE1Position;

        std::unique_ptr<float> min_rad_per_sec_de_0;
        std::unique_ptr<float> min_rad_per_sec_de_1;
        std::unique_ptr<float> max_rad_per_sec_de_0;
        std::unique_ptr<float> max_rad_per_sec_de_1;

        ros::Subscriber jointDEPitchPosSub;
        ros::Subscriber jointDERollPosSub;

        ros::Subscriber throttleSub;
        ros::Subscriber velocitySub;
        ros::Subscriber positionSub;
        ros::Subscriber armHWJointDataSub;
    };

} // namespace mrover
