#include "ControllerMap.h"

void ControllerMap::init(XmlRpc::XmlRpcValue& root) {
    int32_t size = root.size();
    for (int32_t i = 0; i < size; ++i) {
        assert(root[i].hasMember("name") &&
               root[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string name = static_cast<std::string>(root[i]["name"]);

        assert(root[i].hasMember("driver_voltage") &&
               root[i]["driver_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto driverVoltage = (float) static_cast<double>(root[i]["driver_voltage"]);

        assert(root[i].hasMember("motor_max_voltage") &&
               root[i]["motor_max_voltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto motorMaxVoltage = (float) static_cast<double>(root[i]["motor_max_voltage"]);

        float voltageMultiplier = 1.0f;
        if (root[i].hasMember("voltage_multiplier") &&
            root[i]["voltage_multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            voltageMultiplier = (float) static_cast<double>(root[i]["voltage_multiplier"]);
        }
        motorMaxVoltage *= voltageMultiplier;

        assert(root[i].hasMember("mcu_id") &&
               root[i]["mcu_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        auto mcu_id = (uint8_t) static_cast<int>(root[i]["mcu_id"]);

        assert(root[i].hasMember("motor_id") &&
               root[i]["motor_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        auto motor_id = (uint8_t) static_cast<int>(root[i]["motor_id"]);

        controllersByName[name] =
                new Controller(name, mcu_id, motor_id, motorMaxVoltage, driverVoltage);

        if (root[i].hasMember("quad_cpr") &&
            root[i]["quad_cpr"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->quadCPR = (float) static_cast<double>(root[i]["quad_cpr"]);
        }
        if (root[i].hasMember("kP") &&
            root[i]["kP"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kP = (float) static_cast<double>(root[i]["kP"]);
        }
        if (root[i].hasMember("kI") &&
            root[i]["kI"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kI = (float) static_cast<double>(root[i]["kI"]);
        }
        if (root[i].hasMember("kD") &&
            root[i]["kD"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kD = (float) static_cast<double>(root[i]["kD"]);
        }
        if (root[i].hasMember("inversion") &&
            root[i]["inversion"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->inversion = (float) static_cast<double>(root[i]["inversion"]);
        }
        if (root[i].hasMember("limit_a_present") &&
            root[i]["limit_a_present"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            controllersByName[name]->limitAPresent = static_cast<bool>(root[i]["limit_a_present"]);
        }
        if (root[i].hasMember("limit_b_present") &&
            root[i]["limit_b_present"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            controllersByName[name]->limitBPresent = static_cast<bool>(root[i]["limit_b_present"]);
        }
        if (root[i].hasMember("active_limit_a") &&
            root[i]["active_limit_a"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            controllersByName[name]->limitAIsActiveHigh = static_cast<bool>(root[i]["active_limit_a"]);
        }
        if (root[i].hasMember("active_limit_b") &&
            root[i]["active_limit_b"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            controllersByName[name]->limitBIsActiveHigh = static_cast<bool>(root[i]["active_limit_b"]);
        }
        if (root[i].hasMember("counts_limit_a") &&
            root[i]["counts_limit_a"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            controllersByName[name]->limitAAdjustedCounts = static_cast<int>(root[i]["counts_limit_a"]);
        }
        if (root[i].hasMember("counts_limit_b") &&
            root[i]["counts_limit_b"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            controllersByName[name]->limitBAdjustedCounts = static_cast<int>(root[i]["counts_limit_b"]);
        }
        if (root[i].hasMember("limit_a_is_fwd") &&
            root[i]["limit_a_is_fwd"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            controllersByName[name]->limitAIsFwd = static_cast<bool>(root[i]["limit_a_is_fwd"]);
        }
        if (root[i].hasMember("calibration_vel") &&
            root[i]["calibration_vel"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->calibrationVel = (float) static_cast<double>(root[i]["calibration_vel"]);
        }
        ROS_INFO("Made virtual Controller %s on MCU ID %i motor ID %i \n", name.c_str(), mcu_id, motor_id);
    }
}