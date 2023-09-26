#pragma once

#include "controller.hpp"

#include <map>

class MotorsManager {
public:
    MotorsManager(ros::NodeHandle* n, const std::vector<std::string>& controllerNames, XmlRpc::XmlRpcValue root) {
        for (const std::string& name: controllerNames) {
            assert(root[name].hasMember("type") &&
               root[name]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string type = static_cast<std::string>(root[name]["type"]);
            assert(type == "brushed" || type == "brushless");
            if (type == "brushed") {
                m[name] = std::make_unique<BrushedController>(&CANPublisher);    
            }
            else if (type == "brushless") {
                m[name] = std::make_unique<BrushlessController>(&CANPublisher);
            }
        }
    }

    auto& get_controller(std::string const& name) {
        return m.at(name);
    }

    virtual void process_frame(uint64_t frame) = 0;

private:
    std::map<std::string, std::unique_ptr<Controller>> m;
};