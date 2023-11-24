#pragma once

#include <unordered_map>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <brushed.hpp>
#include <brushless.hpp>
#include <controller.hpp>

#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>

namespace mrover {

    template<IsUnit Unit>
    auto requireParamAsUnit(ros::NodeHandle const& nh, std::string const& name) -> Unit {
        assert(nh.hasParam(name));

        typename Unit::rep_t value;
        nh.getParam(name, value);
        return Unit{value};
    }

    class MotorsManager {
    public:
        MotorsManager() = default;

        MotorsManager(ros::NodeHandle const& nh, std::string groupName, std::vector<std::string> controllerNames);

        Controller& get_controller(std::string const& name);

        void moveMotorsThrottle(Throttle::ConstPtr const& msg);

        void moveMotorsVelocity(Velocity::ConstPtr const& msg);

        void moveMotorsPosition(Position::ConstPtr const& msg);

        void publishDataCallback(ros::TimerEvent const&);

    private:
        ros::NodeHandle mNh;

        ros::Subscriber mMoveThrottleSub;
        ros::Subscriber mMoveVelocitySub;
        ros::Subscriber mMovePositionSub;
        ros::Publisher mJointDataPub;
        ros::Publisher mControllerDataPub;
        std::unordered_map<std::string, std::unique_ptr<Controller>> mControllers;
        std::string mGroupName;
        std::vector<std::string> mControllerNames;
        ros::Timer publishDataTimer;
    };

} // namespace mrover
