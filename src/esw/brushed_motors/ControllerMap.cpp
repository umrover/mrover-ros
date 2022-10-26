#include "ControllerMap.h"

void ControllerMap::init(XmlRpc::XmlRpcValue& root) {
    for (auto &i : root) {
        assert(i.second.hasMember("name") &&
               i.second["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        auto name = static_cast<std::string>(i.second["name"]);

        assert(i.second.hasMember("driver_voltage") &&
               i.second["driver_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto driverVoltage = (float) static_cast<double>(i.second["driver_voltage"]);

        assert(i.second.hasMember("motor_max_voltage") &&
               i.second["motor_max_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto motorMaxVoltage = (float) static_cast<double>(i.second["motor_max_voltage"]);

        float voltageMultiplier = 1.0f;
        if (i.second.hasMember("voltage_multiplier") &&
            i.second["voltage_multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            voltageMultiplier = (float) static_cast<double>(i.second["voltage_multiplier"]);
        }
        motorMaxVoltage *= voltageMultiplier;

        assert(i.second.hasMember("nucleo") &&
               i.second["nucleo"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        auto nucleo = (uint8_t) static_cast<int>(i.second["nucleo"]);

        assert(i.second.hasMember("channel") &&
               i.second["channel"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        auto channel = (uint8_t) static_cast<int>(i.second["channel"]);

        uint8_t calculatedAddress = (nucleo << 4) | channel;
        controllersByName[name] =
                new Controller(name, calculatedAddress, motorMaxVoltage, driverVoltage);

        if (i.second.hasMember("quad_cpr") &&
            i.second["quad_cpr"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->quadCPR = (float) static_cast<double>(i.second["quad_cpr"]);
        }
        if (i.second.hasMember("kP") &&
            i.second["kP"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kP = (float) static_cast<double>(i.second["kP"]);
        }
        if (i.second.hasMember("kI") &&
            i.second["kI"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kI = (float) static_cast<double>(i.second["kI"]);
        }
        if (i.second.hasMember("kD") &&
            i.second["kD"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kD = (float) static_cast<double>(i.second["kD"]);
        }
        if (i.second.hasMember("inversion") &&
            i.second["inversion"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->inversion = (float) static_cast<double>(i.second["inversion"]);
        }
        ROS_INFO("Made virtual Controller %s on Nucleo %i channel %i \n", name.c_str(), nucleo, channel);
    }
}