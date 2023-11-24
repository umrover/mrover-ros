#include "motors_manager.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    MotorsManager::MotorsManager(ros::NodeHandle const& nh, std::string groupName, std::vector<std::string> controllerNames)
        : mNh{nh},
          mControllerNames{std::move(controllerNames)},
          mGroupName{std::move(groupName)} {

        // Load motor controllers configuration from the ROS parameter server
        XmlRpc::XmlRpcValue controllersRoot;
        assert(nh.hasParam("motors/controllers"));
        nh.getParam("motors/controllers", controllersRoot);
        assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        for (std::string const& name: mControllerNames) {
            auto type = xmlRpcValueToTypeOrDefault<std::string>(controllersRoot[name], "type");
            assert(type == "brushed" || type == "brushless");

            // TODO: avoid hard coding Jetson here - can move into constructor of MotorsManager
            // and let the bridge nodes hardcode as jetson.
            if (type == "brushed") {
                auto temp = std::make_unique<BrushedController>(nh, "jetson", name);
                mControllers[name] = std::move(temp);
            } else if (type == "brushless") {
                auto temp = std::make_unique<BrushlessController>(nh, "jetson", name);
                mControllers[name] = std::move(temp);
            }
        }


        // Subscribe to the ROS topic for commands
        mMoveThrottleSub = mNh.subscribe<Throttle>(std::format("{}_throttle_cmd", mGroupName), 1, &MotorsManager::moveMotorsThrottle, this);
        mMoveVelocitySub = mNh.subscribe<Velocity>(std::format("{}_velocity_cmd", mGroupName), 1, &MotorsManager::moveMotorsVelocity, this);
        mMovePositionSub = mNh.subscribe<Position>(std::format("{}_position_cmd", mGroupName), 1, &MotorsManager::moveMotorsPosition, this);

        mJointDataPub = mNh.advertise<sensor_msgs::JointState>(std::format("{}_joint_data", mGroupName), 1);
        mControllerDataPub = mNh.advertise<ControllerState>(std::format("{}_controller_data", mGroupName), 1);

        publishDataTimer = mNh.createTimer(ros::Duration(0.1), &MotorsManager::publishDataCallback, this);
    }

    Controller& MotorsManager::get_controller(std::string const& name) {
        return *mControllers.at(name);
    }
    void MotorsManager::moveMotorsThrottle(Throttle::ConstPtr const& msg) {
        for (size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Throttle request for %s ignored!", name.c_str());
                continue;
            }
            Controller& controller = get_controller(name);
            controller.setDesiredThrottle(msg->throttles[i]);
        }
    }

    void MotorsManager::moveMotorsVelocity(Velocity::ConstPtr const& msg) {
        for (size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Velocity request for %s ignored!", name.c_str());
                continue;
            }
            Controller& controller = get_controller(name);
            controller.setDesiredVelocity(RadiansPerSecond{msg->velocities[i]});
        }
    }

    void MotorsManager::moveMotorsPosition(Position::ConstPtr const& msg) {
        for (std::size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Position request for %s ignored!", name.c_str());
                continue;
            }
            Controller& controller = get_controller(name);
            controller.setDesiredPosition(Radians{msg->positions[i]});
        }
    }

    void MotorsManager::publishDataCallback(ros::TimerEvent const&) {
        sensor_msgs::JointState joint_state;
        ControllerState controller_state;
        for (std::string const& name: mControllerNames) {
            Controller& controller = get_controller(name);
            joint_state.name.push_back(name);
            joint_state.position.push_back(controller.getCurrentPosition().get());
            joint_state.velocity.push_back(controller.getCurrentVelocity().get());
            joint_state.effort.push_back(controller.getEffort());

            controller_state.name.push_back(name);
            controller_state.state.push_back(controller.getState());
            controller_state.error.push_back(controller.getErrorState());
            uint8_t limit_hit;
            for (int i = 0; i < 4; ++i) {
                limit_hit |= controller.isLimitHit(i) << i;
            }
            controller_state.limit_hit.push_back(limit_hit);
        }

        mJointDataPub.publish(joint_state);
        mControllerDataPub.publish(controller_state);
    }

} // namespace mrover
