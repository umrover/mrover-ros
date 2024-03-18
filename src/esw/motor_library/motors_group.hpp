#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <controller.hpp>

#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <mrover/MotorsAdjust.h>

namespace mrover {

    class MotorsGroup {
    public:
        MotorsGroup() = default;

        MotorsGroup(ros::NodeHandle const& nh, std::string groupName);

        auto getController(std::string const& name) const -> Controller&;

        auto moveMotorsThrottle(Throttle::ConstPtr const& msg) -> void;

        auto moveMotorsVelocity(Velocity::ConstPtr const& msg) -> void;

        auto moveMotorsPosition(Position::ConstPtr const& msg) -> void;

        auto processJointData(sensor_msgs::JointState::ConstPtr const& msg, std::string const& name) -> void;

        auto processControllerData(ControllerState::ConstPtr const& msg, std::string const& name) -> void;

    private:
        ros::NodeHandle mNh;

        ros::Subscriber mMoveThrottleSub;
        ros::Subscriber mMoveVelocitySub;
        ros::Subscriber mMovePositionSub;

        ros::Publisher mJointDataPub;
        ros::Publisher mControllerDataPub;

        std::unordered_map<std::string, ros::Publisher> mThrottlePubsByName;
        std::unordered_map<std::string, ros::Publisher> mVelocityPubsByName;
        std::unordered_map<std::string, ros::Publisher> mPositionPubsByName;
        std::unordered_map<std::string, ros::Subscriber> mJointDataSubsByName;
        std::unordered_map<std::string, ros::Subscriber> mControllerDataSubsByName;
        
        std::unordered_map<std::string, size_t> mIndexByName;

        std::unordered_map<std::string, std::unique_ptr<Controller>> mControllers;
        std::string mGroupName;
        std::vector<std::string> mControllerNames;

        sensor_msgs::JointState mJointState;
        ControllerState mControllerState;
    };

} // namespace mrover
