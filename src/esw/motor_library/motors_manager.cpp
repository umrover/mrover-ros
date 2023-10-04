#include "motors_manager.hpp"

MotorsManager::MotorsManager(ros::NodeHandle& n, const std::string& groupName, const std::vector<std::string>& controllerNames) {
    motorGroupName = groupName;
    motorNames = controllerNames;
    // Load motor controllers configuration from the ROS parameter server
    XmlRpc::XmlRpcValue controllersRoot;
    assert(n.getParam("motors/controllers", controllersRoot));
    assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const std::string& name: controllerNames) {
        assert(controllersRoot[name].hasMember("type") &&
                controllersRoot[name]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string type = static_cast<std::string>(controllersRoot[name]["type"]);
        assert(type == "brushed" || type == "brushless");

        if (type == "brushed") {
            auto temp = std::make_unique<BrushedController>(n, name);
            controllers[name] = std::move(temp);
        } else if (type == "brushless") {
            auto temp = std::make_unique<BrushlessController>(n, name);
            controllers[name] = std::move(temp);
        }

        names[controllers[name]->get_can_manager().get_id()] = name;
    }

    updateLastConnection();

    // Subscribe to the ROS topic for commands
    ros::Subscriber moveThrottleSub = n.subscribe<mrover::Throttle>(groupName + "_throttle_cmd", 1, &MotorsManager::moveMotorsThrottle, this);
    ros::Subscriber moveVelocitySub = n.subscribe<mrover::Velocity>(groupName + "_velocity_cmd", 1, &MotorsManager::moveMotorsVelocity, this);
    ros::Subscriber movePositionSub = n.subscribe<mrover::Position>(groupName + "_position_cmd", 1, &MotorsManager::moveMotorsPosition, this);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = n.createTimer(ros::Duration(0.1), &MotorsManager::heartbeatCallback, this);
}

Controller& MotorsManager::get_controller(std::string const& name) {
    return *controllers.at(name);
}

void MotorsManager::process_frame(int bus, int id, uint64_t frame_data) {
    // TODO: figure out how to organize by bus
    controllers[names[bus | (id << 4)]]->update(frame_data);
}

void MotorsManager::moveMotorsThrottle(const mrover::Throttle::ConstPtr& msg) {
    if (msg->names != motorNames && msg->names.size() != msg->throttles.size()) {
        ROS_ERROR("Throttle request is invalid!");
        return;
    }

    updateLastConnection();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = get_controller(name);
        controller.set_desired_throttle(msg->throttles[i]);
    }
}

void MotorsManager::moveMotorsVelocity(const mrover::Velocity::ConstPtr& msg) {
    if (msg->names != motorNames && msg->names.size() != msg->velocities.size()) {
        ROS_ERROR("Velocity request is invalid!");
        return;
    }

    updateLastConnection();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = get_controller(name);
        controller.set_desired_velocity(msg->velocities[i]);
    }
}

void MotorsManager::moveMotorsPosition(const mrover::Position::ConstPtr& msg) {
    if (msg->names != motorNames && msg->names.size() != msg->positions.size()) {
        ROS_ERROR("Arm request is invalid!");
        return;
    }

    updateLastConnection();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = get_controller(name);
        controller.set_desired_position(msg->positions[i]);
    }
}

void MotorsManager::heartbeatCallback(const ros::TimerEvent&) {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastConnection);
    if (duration.count() < 100) {
        for (const auto& motorName: motorNames) {
            Controller& controller = get_controller(motorName);
            controller.set_desired_throttle(0.0);
        }
    }
}

void MotorsManager::updateLastConnection() {
    // Used as an override if we know that all motors have been called recently.
    lastConnection = std::chrono::high_resolution_clock::now();
}