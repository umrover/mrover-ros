#include "motors_manager.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    MotorsManager::MotorsManager(ros::NodeHandle const& nh, std::string groupName)
        : mNh{nh},
          mGroupName{std::move(groupName)} {

        XmlRpc::XmlRpcValue motorControllerNames;
        assert(mNh.hasParam(std::format("motors_groups/{}", mGroupName)));
        mNh.getParam(std::format("motors_groups/{}", mGroupName), motorControllerNames);
        assert(motorControllerNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (auto const& [name, type]: motorControllerNames) {
            assert(type.getType() == XmlRpc::XmlRpcValue::TypeString);
            mIndexByName[name] = mControllerNames.size();
            mControllerNames.push_back(static_cast<std::string>(name));
            mThrottlePubsByName[name] = mNh.advertise<Throttle>(std::format("{}_throttle_cmd", name), 1);
            mVelocityPubsByName[name] = mNh.advertise<Velocity>(std::format("{}_velocity_cmd", name), 1);
            mPositionPubsByName[name] = mNh.advertise<Position>(std::format("{}_position_cmd", name), 1);
            mJointDataSubsByName[name] = mNh.subscribe<sensor_msgs::JointState>(
                    std::format("{}_joint_data", name), 1,
                    [name, this](sensor_msgs::JointState::ConstPtr const& msg) {
                        return MotorsManager::processJointData(msg, name);
                    });
            mControllerDataSubsByName[name] = mNh.subscribe<ControllerState>(
                    std::format("{}_controller_data", name), 1,
                    [name, this](ControllerState::ConstPtr const& msg) {
                        return MotorsManager::processControllerData(msg, name);
                    });
            mJointState.name.push_back(name);
            mControllerState.name.push_back(name);
        }
        mJointState.position.resize(mJointState.name.size());
        mJointState.velocity.resize(mJointState.name.size());
        mJointState.effort.resize(mJointState.name.size());
        mControllerState.state.resize(mControllerState.name.size());
        mControllerState.error.resize(mControllerState.name.size());
        mControllerState.limit_hit.resize(mControllerState.name.size());

        // Load motor controllers configuration from the ROS parameter server
        XmlRpc::XmlRpcValue controllersRoot;
        assert(mNh.hasParam("motors/controllers"));
        mNh.getParam("motors/controllers", controllersRoot);
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

            Throttle throttle;
            throttle.names = {name};
            throttle.throttles = {msg->throttles[i]};
            mThrottlePubsByName[name].publish(throttle);
        }
    }

    void MotorsManager::moveMotorsVelocity(Velocity::ConstPtr const& msg) {
        for (size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Velocity request for %s ignored!", name.c_str());
                continue;
            }


            Velocity velocity;
            velocity.names = {name};
            velocity.velocities = {msg->velocities[i]};
            mVelocityPubsByName[name].publish(velocity);
        }
    }

    void MotorsManager::moveMotorsPosition(Position::ConstPtr const& msg) {
        for (std::size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Position request for %s ignored!", name.c_str());
                continue;
            }

            Position position;
            position.names = {name};
            position.positions = {msg->positions[i]};
            mPositionPubsByName[name].publish(position);
        }
    }

    void MotorsManager::processJointData(sensor_msgs::JointState::ConstPtr const& msg, std::string const& name) {
        if (msg->name.size() != 1 || msg->position.size() != 1 || msg->velocity.size() != 1 || msg->effort.size() != 1) {
            ROS_ERROR("Process joint data for %s ignored!", name.c_str());
            return;
        }

        size_t index = mIndexByName.at(name);

        mJointState.position[index] = msg->position[0];
        mJointState.velocity[index] = msg->velocity[0];
        mJointState.effort[index] = msg->effort[0];

        mJointDataPub.publish(mJointState);
    }

    void MotorsManager::processControllerData(ControllerState::ConstPtr const& msg, std::string const& name) {
        if (msg->name.size() != 1 || msg->state.size() != 1 || msg->error.size() != 1 || msg->limit_hit.size() != 1) {
            ROS_ERROR("Process controller data for %s ignored!", name.c_str());
            return;
        }

        auto index = mIndexByName.at(name);

        mControllerState.state[index] = msg->state[0];
        mControllerState.error[index] = msg->error[0];
        mControllerState.limit_hit[index] = msg->limit_hit[0];

        mControllerDataPub.publish(mControllerState);
    }

} // namespace mrover
