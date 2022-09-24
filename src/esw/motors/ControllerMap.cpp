#include "ControllerMap.h"

void ControllerMap::init(XmlRpc::XmlRpcValue& root) {
    for (int32_t i = 0; i < root.size(); ++i) {
        assert(root[i].hasMember("name") &&
               root[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string name = static_cast<std::string>(root[i]["name"]);

        assert(root[i].hasMember("inputVoltage") &&
               root[i]["inputVoltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double input_voltage = static_cast<double>(root[i]["inputVoltage"]);

        assert(root[i].hasMember("outputVoltage") &&
               root[i]["outputVoltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double output_voltage = static_cast<double>(root[i]["outputVoltage"]);

        double percentage = 100.0 * input_voltage / output_voltage;
        if (root[i].hasMember("voltageMultiplier") &&
            root[i]["voltageMultiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            percentage *= static_cast<double>(root[i]["voltageMultiplier"]);
        }

        uint16_t pwm_max = static_cast<int>(percentage);
        pwm_max = pwm_max > 100 ? 100 : pwm_max;
        pwm_max = pwm_max < 0 ? 0 : pwm_max;

        assert(root[i].hasMember("nucleo") &&
               root[i]["nucleo"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t nucleo = static_cast<uint8_t>(root[i]["nucleo"]);

        assert(root[i].hasMember("channel") &&
               root[i]["channel"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t channel = static_cast<uint8_t>(root[i]["channel"]);

        controllers[name] = new Controller(name, pwm_max);
        name_map[name] = calculate_i2c_address(nucleo, channel);

        if (root[i].hasMember("quadCPR") &&
            root[i]["quadCPR"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->quad_cpr = static_cast<float>(root[i]["quadCPR"]);
        }
        if (root[i].hasMember("kP") &&
            root[i]["kP"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kP = static_cast<float>(root[i]["kP"]);
        }
        if (root[i].hasMember("kI") &&
            root[i]["kI"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kI = static_cast<float>(root[i]["kI"]);
        }
        if (root[i].hasMember("kD") &&
            root[i]["kD"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->kD = static_cast<float>(root[i]["kD"]);
        }
        if (root[i].hasMember("inversion") &&
            root[i]["inversion"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllers[name]->inversion = static_cast<float>(root[i]["inversion"]);
        }
        printf("Virtual Controller %s on Nucleo %i channel %i \n", name.c_str(), nucleo, channel);
    }
}