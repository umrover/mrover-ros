#include "ControllerMap.h"
#include "Controller.h"

// Helper function to calculate an i2c address based off of nucleo # and channel #
uint8_t ControllerMap::calculate_i2c_address(uint8_t nucleo, uint8_t channel) {
    return (nucleo << 4) | channel;
}

// Initialization function
void ControllerMap::init() {
    XmlRpc::XmlRpcValue root;
    ros::param::get("motors/controllers", root);
    for (int32_t i = 0; i < root.size(); ++i) {
        assert(root[i].hasMember("name") && root[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string name = static_cast<std::string>(root[i]["name"]);

        assert(root[i].hasMember("input_voltage") && root[i]["input_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double input_voltage = static_cast<double>(root[i]["input_voltage"]);

        assert(root[i].hasMember("output_voltage") && root[i]["output_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double output_voltage = static_cast<double>(root[i]["output_voltage"]);

        double percentage = 100.0 * input_voltage / output_voltage;
        if (root[i].hasMember("voltage_multiplier") && root[i]["voltage_multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            percentage *= static_cast<double>(root[i]["voltage_multiplier"]);
        }

        uint16_t pwm_max = static_cast<int>(percentage);
        pwm_max = pwm_max > 100 ? 100 : pwm_max;
        pwm_max = pwm_max < 0 ? 0 : pwm_max;

        assert(root[i].hasMember("nucleo") && root[i]["nucleo"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t nucleo = static_cast<uint8_t>(root[i]["nucleo"]);

        assert(root[i].hasMember("channel") && root[i]["channel"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t channel = static_cast<uint8_t>(root[i]["channel"]);

        controllers[name] = new Controller(name, pwm_max);
        name_map[name] = calculate_i2c_address(nucleo, channel);

        if (root[i].hasMember("quadCPR") && root[i]["quadCPR"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->quad_cpr = static_cast<float>(root[i]["quadCPR"]);
        }
        if (root[i].hasMember("kP") && root[i]["kP"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kP = static_cast<float>(root[i]["kP"]);
        }
        if (root[i].hasMember("kI") && root[i]["kI"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kI = static_cast<float>(root[i]["kI"]);
        }
        if (root[i].hasMember("kD") && root[i]["kD"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kD = static_cast<float>(root[i]["kD"]);
        }
        if (root[i].hasMember("inversion") && root[i]["inversion"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->inversion = static_cast<float>(root[i]["inversion"]);
        }
        printf("Virtual Controller %s on Nucleo %i channel %i \n", name.c_str(), nucleo, channel);
    }
}

// Returns supposed i2c address based off of virtual controller name
uint8_t ControllerMap::get_i2c_address(std::string name) {
    return name_map[name];
}

// Returns whether virtual controller name is in the "live" virtual controller to i2c address map
bool ControllerMap::check_if_live(std::string name) {
    return (name == live_map[get_i2c_address(name)]);
}

// Forces this virtual controller into the i2c address to "live" virtual controller map, replacing any virtual controller already at that i2c address
void ControllerMap::make_live(std::string name) {
    live_map[get_i2c_address(name)] = name;
}
