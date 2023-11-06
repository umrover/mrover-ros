#include "motors_manager.hpp"

namespace mrover {

    MotorsManager::MotorsManager(ros::NodeHandle& nh, const std::string& groupName, const std::vector<std::string>& controllerNames) {
        motorGroupName = groupName;
        motorNames = controllerNames;
        // Load motor controllers configuration from the ROS parameter server
        XmlRpc::XmlRpcValue controllersRoot;
        assert(nh.hasParam("motors/controllers"));
        nh.getParam("motors/controllers", controllersRoot);
        assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        for (const std::string& name: controllerNames) {
            assert(controllersRoot[name].hasMember("type") &&
                   controllersRoot[name]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string type = static_cast<std::string>(controllersRoot[name]["type"]);
            assert(type == "brushed" || type == "brushless");

            if (type == "brushed") {
                auto temp = std::make_unique<BrushedController>(nh, name);
                controllers[name] = std::move(temp);
            } else if (type == "brushless") {
                auto temp = std::make_unique<BrushlessController>(nh, name);
                controllers[name] = std::move(temp);
            }

            // TODO(guthrie) make this compile
            //            names[controllers[name]->get_can_manager().get_id()] = name;
        }

        updateLastConnection();

        // Subscribe to the ROS topic for commands
        ros::Subscriber moveThrottleSub = nh.subscribe<Throttle>(groupName + "_throttle_cmd", 1, &MotorsManager::moveMotorsThrottle, this);
        ros::Subscriber moveVelocitySub = nh.subscribe<Velocity>(groupName + "_velocity_cmd", 1, &MotorsManager::moveMotorsVelocity, this);
        ros::Subscriber movePositionSub = nh.subscribe<Position>(groupName + "_position_cmd", 1, &MotorsManager::moveMotorsPosition, this);

        // Create a 0.1 second heartbeat timer
        ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), &MotorsManager::heartbeatCallback, this);
    }

    Controller& MotorsManager::get_controller(std::string const& name) {
        return *controllers.at(name);
    }

    void MotorsManager::process_frame(int bus, int id, std::span<std::byte const> frame_data) {
        // TODO: figure out how to send to corresponding controller
    }

    void MotorsManager::moveMotorsThrottle(const Throttle::ConstPtr& msg) {
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

    void MotorsManager::moveMotorsVelocity(const Velocity::ConstPtr& msg) {
        if (msg->names != motorNames && msg->names.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity request is invalid!");
            return;
        }

        updateLastConnection();

        for (size_t i = 0; i < msg->names.size(); ++i) {
            const std::string& name = msg->names[i];
            Controller& controller = get_controller(name);
            controller.set_desired_velocity(RadiansPerSecond{msg->velocities[i]});
        }
    }

    void MotorsManager::moveMotorsPosition(const Position::ConstPtr& msg) {
        if (msg->names != motorNames && msg->names.size() != msg->positions.size()) {
            ROS_ERROR("Arm request is invalid!");
            return;
        }

        updateLastConnection();

        for (size_t i = 0; i < msg->names.size(); ++i) {
            const std::string& name = msg->names[i];
            Controller& controller = get_controller(name);
            controller.set_desired_position(Radians{msg->positions[i]});
        }
    }

    void MotorsManager::heartbeatCallback(const ros::TimerEvent&) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastConnection);
        if (duration.count() < 100) {
            for (const auto& motorName: motorNames) {
                Controller& controller = get_controller(motorName);
                controller.set_desired_throttle(0_percent);
            }
        }
    }

    void MotorsManager::updateLastConnection() {
        // Used as an override if we know that all motors have been called recently.
        lastConnection = std::chrono::high_resolution_clock::now();
    }

} // namespace mrover
