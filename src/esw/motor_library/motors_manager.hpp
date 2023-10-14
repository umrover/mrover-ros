#pragma once

#include <chrono>
#include <unordered_map>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <brushed.hpp>
#include <brushless.hpp>
#include <controller.hpp>

#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>

namespace mrover {

    template<IsUnit Unit>
    Unit requireParamAsUnit(ros::NodeHandle const& nh, std::string const& name) {
        assert(nh.hasParam(name));

        typename Unit::rep_t value;
        nh.getParam(name, value);
        return make_unit<Unit>(value);
    }

    class MotorsManager {
    public:
        MotorsManager() = default;

        MotorsManager(ros::NodeHandle& nh, const std::string& groupName, const std::vector<std::string>& controllerNames);

        Controller& get_controller(std::string const& name);

        void process_frame(int bus, int id, std::span<std::byte const> frame_data);

        void moveMotorsThrottle(const Throttle::ConstPtr& msg);

        void moveMotorsVelocity(const Velocity::ConstPtr& msg);

        void moveMotorsPosition(const Position::ConstPtr& msg);

        void heartbeatCallback(const ros::TimerEvent&);

        void updateLastConnection();

    private:
        std::unordered_map<std::string, std::unique_ptr<Controller>> controllers;
        std::unordered_map<int, std::string> names;
        std::string motorGroupName;
        std::vector<std::string> motorNames;
        std::chrono::high_resolution_clock::time_point lastConnection;
    };

} // namespace mrover
