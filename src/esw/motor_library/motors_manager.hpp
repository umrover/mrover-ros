#pragma once

#include "controller.hpp"
#include <XmlRpcValue.h>
#include <unordered_map>
#include <ros/ros.h>

class MotorsManager {
public:
    MotorsManager(ros::NodeHandle* n, const std::vector<std::string>& controllerNames, XmlRpc::XmlRpcValue root) {
        for (const std::string& name: controllerNames) {
            assert(root[name].hasMember("type") &&
                   root[name]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string type = static_cast<std::string>(root[name]["type"]);
            assert(type == "brushed" || type == "brushless");

            if (type == "brushed") {
                controllers[name] = std::make_unique<BrushedController>(n, name);
            } else if (type == "brushless") {
                controllers[name] = std::make_unique<BrushlessController>(n, name);
            }

            names[controllers[name]->get_can_manager().get_id()] = name;
        }
    }

    auto& get_controller(std::string const& name) {
        return controllers.at(name);
    }

    void process_frame(int bus, int id, uint64_t frame_data) {
        // TODO: figure out how to organize by bus
        controllers[names[bus | (id << 4)]]->update(frame_data);
    }

private:
    std::unordered_map<std::string, std::unique_ptr<Controller>> controllers;
    std::unordered_map<int, std::string> names;
};