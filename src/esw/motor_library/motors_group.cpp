#include "motors_group.hpp"

#include <brushed.hpp>
#include <brushless.hpp>

#include <params_utils.hpp>

namespace mrover {

    using namespace std::chrono_literals;

    MotorsGroup::MotorsGroup(ros::NodeHandle const& nh, std::string groupName)
        : mNh{nh},
          mGroupName{std::move(groupName)} {

        XmlRpc::XmlRpcValue motorControllerNames;
        assert(mNh.hasParam(std::format("motors_group/{}", mGroupName)));
        mNh.getParam(std::format("motors_group/{}", mGroupName), motorControllerNames);
        assert(motorControllerNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < motorControllerNames.size(); ++i) { // NOLINT(*-loop-convert)
            auto name = static_cast<std::string>(motorControllerNames[i]);
            mIndexByName[name] = mControllerNames.size();
            mControllerNames.push_back(name);
            mThrottlePubsByName[name] = mNh.advertise<Throttle>(std::format("{}_throttle_cmd", name), 1);
            mVelocityPubsByName[name] = mNh.advertise<Velocity>(std::format("{}_velocity_cmd", name), 1);
            mPositionPubsByName[name] = mNh.advertise<Position>(std::format("{}_position_cmd", name), 1);
            mJointDataSubsByName[name] = mNh.subscribe<sensor_msgs::JointState>(
                    std::format("{}_joint_data", name), 1,
                    [name, this](sensor_msgs::JointState::ConstPtr const& msg) {
                        return processJointData(msg, name);
                    });
            mControllerDataSubsByName[name] = mNh.subscribe<ControllerState>(
                    std::format("{}_controller_data", name), 1,
                    [name, this](ControllerState::ConstPtr const& msg) {
                        return processControllerData(msg, name);
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
            if (!controllersRoot.hasMember(name)) {
                ROS_ERROR_STREAM(std::format("There is a mismatch in the config - motor {} doesn't exist!", name));
                throw;
            }

            auto type = xmlRpcValueToTypeOrDefault<std::string>(controllersRoot[name], "type");
            if (!(type == "brushed" || type == "brushless" || type == "brushless_linear")) {
                ROS_ERROR_STREAM(std::format("Unknown motor type %s!", type));
                throw;
            }

            // TODO: avoid hard coding Jetson here - can move into constructor of MotorsGroup
            // and let the bridge nodes hardcode as jetson.
            if (type == "brushed") {
                mControllers.try_emplace(name, std::in_place_type<BrushedController>, mNh, "jetson", name);
            } else if (type == "brushless") {
                mControllers.try_emplace(name, std::in_place_type<BrushlessController<Revolutions>>, mNh, "jetson", name);
            } else if (type == "brushless_linear") {
                mControllers.try_emplace(name, std::in_place_type<BrushlessController<Meters>>, mNh, "jetson", name);
            } else {
                ROS_ERROR_STREAM(std::format("Unknown motor type %s!", type));
                throw;
            }
        }

        mMoveThrottleSub = mNh.subscribe<Throttle>(std::format("{}_throttle_cmd", mGroupName), 1, &MotorsGroup::moveMotorsThrottle, this);
        mMoveVelocitySub = mNh.subscribe<Velocity>(std::format("{}_velocity_cmd", mGroupName), 1, &MotorsGroup::moveMotorsVelocity, this);
        mMovePositionSub = mNh.subscribe<Position>(std::format("{}_position_cmd", mGroupName), 1, &MotorsGroup::moveMotorsPosition, this);

        mControllerDataPub = mNh.advertise<ControllerState>(std::format("{}_controller_data", mGroupName), 1);
        mJointDataPub = mNh.advertise<sensor_msgs::JointState>(std::format("{}_joint_data", mGroupName), 1);
    }

    auto MotorsGroup::getController(std::string const& name) -> Controller& {
        return mControllers.at(name);
    }

    auto MotorsGroup::moveMotorsThrottle(Throttle::ConstPtr const& msg) -> void {
        for (size_t i = 0; i < msg->names.size(); ++i) {
            std::string const& name = msg->names[i];
            if (!mControllers.contains(name)) {
                ROS_ERROR("Group throttle request for %s ignored (%f)!", name.c_str(), msg->throttles[i]);
                continue;
            }

            Throttle throttle;
            throttle.names = {name};
            throttle.throttles = {msg->throttles[i]};
            mThrottlePubsByName[name].publish(throttle);
        }
    }

    void MotorsGroup::moveMotorsVelocity(Velocity::ConstPtr const& msg) {
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

    void MotorsGroup::moveMotorsPosition(Position::ConstPtr const& msg) {
        // TODO - if any of the motor positions are invalid, then u should cancel the message.

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

    void MotorsGroup::processJointData(sensor_msgs::JointState::ConstPtr const& msg, std::string const& name) {
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

    void MotorsGroup::processControllerData(ControllerState::ConstPtr const& msg, std::string const& name) {
        if (msg->name.size() != 1 || msg->state.size() != 1 || msg->error.size() != 1 || msg->limit_hit.size() != 1) {
            ROS_ERROR("Process controller data for %s ignored!", name.c_str());
            return;
        }

        std::size_t index = mIndexByName.at(name);

        mControllerState.state[index] = msg->state[0];
        mControllerState.error[index] = msg->error[0];
        mControllerState.limit_hit[index] = msg->limit_hit[0];

        mControllerDataPub.publish(mControllerState);
    }

} // namespace mrover
