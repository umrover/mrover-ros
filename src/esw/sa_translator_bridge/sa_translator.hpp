#pragma once

#include <format>
#include <memory>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include "units/units.hpp"

#include <read_from_ros_param.hpp>
#include <linear_joint_translation.hpp>
#include <mrover/AdjustMotor.h>
#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>


namespace mrover {

    class SATranslator {
    public:
        SATranslator() = default;

        SATranslator(ros::NodeHandle& nh);

        void processVelocityCmd(Velocity::ConstPtr const& msg);

        void processPositionCmd(Position::ConstPtr const& msg);

        void processThrottleCmd(Throttle::ConstPtr const& msg);

        void processSAHWJointData(sensor_msgs::JointState::ConstPtr const& msg);

    private:

        const std::vector<std::string> mSAHWNames = {"sa_x", "sa_y", "sa_z", "sampler", "sensor_actuator"};
        std::unique_ptr<ros::Publisher> mThrottlePub;
        std::unique_ptr<ros::Publisher> mVelocityPub;
        std::unique_ptr<ros::Publisher> mPositionPub;
        std::unique_ptr<ros::Publisher> mJointDataPub;
       
        const size_t mXAxisIndex = std::find(mSAHWNames.begin(), mSAHWNames.end(), "sa_x") - mSAHWNames.begin();
        const size_t mYAxisIndex = std::find(mSAHWNames.begin(), mSAHWNames.end(), "sa_x") - mSAHWNames.begin();
        const size_t mZAxisIndex = std::find(mSAHWNames.begin(), mSAHWNames.end(), "sa_x") - mSAHWNames.begin();

        RadiansPerMeter mXAxisMult{};
        RadiansPerMeter mYAxisMult{};
        RadiansPerMeter mZAxisMult{};

        ros::Subscriber mThrottleSub;
        ros::Subscriber mVelocitySub;
        ros::Subscriber mPositionSub;
        ros::Subscriber mSAHWJointDataSub;

    };

} // namespace mrover
