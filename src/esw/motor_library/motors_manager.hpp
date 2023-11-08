#pragma once

#include <chrono>
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

        void process_frame(int bus, int id, std::span<std::byte const> frame_data);

        void moveMotorsThrottle(const Throttle::ConstPtr& msg);

        void moveMotorsVelocity(const Velocity::ConstPtr& msg);

        void moveMotorsPosition(const Position::ConstPtr& msg);

        void heartbeatCallback(const ros::TimerEvent&);

        void publishDataCallback(const ros::TimerEvent&);

        void updateLastConnection();

    private:
        ros::NodeHandle mNh;

        ros::Subscriber mMoveThrottleSub;
        ros::Subscriber mMoveVelocitySub;
        ros::Subscriber mMovePositionSub;
        ros::Publisher mJointDataPub;
        ros::Publisher mControllerDataPub;
        // TODO - create a publisher and add to ESW TELEOP ICD about limit switch hit stuff
        std::unordered_map<std::string, std::unique_ptr<Controller>> mControllers;
        std::unordered_map<int, std::string> mNames;
        std::string mGroupName;
        std::vector<std::string> mControllerNames;
        std::chrono::high_resolution_clock::time_point lastConnection;
    };

} // namespace mrover
