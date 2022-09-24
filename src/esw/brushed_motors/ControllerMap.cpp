#include "ControllerMap.h"

void ControllerMap::init(XmlRpc::XmlRpcValue& root) {
    for (int32_t i = 0; i < root.size(); ++i) {
        assert(root[i].hasMember("name") &&
               root[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string name = static_cast<std::string>(root[i]["name"]);

        assert(root[i].hasMember("motorMaxVoltage") &&
               root[i]["motorMaxVoltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double motorMaxVoltage = static_cast<double>(root[i]["motorMaxVoltage"]);

        assert(root[i].hasMember("driverVoltage") &&
               root[i]["driverVoltage"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double driverVoltage = static_cast<double>(root[i]["driverVoltage"]);

        assert(root[i].hasMember("nucleo") &&
               root[i]["nucleo"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t nucleo = (uint8_t)static_cast<int>(root[i]["nucleo"]);

        assert(root[i].hasMember("channel") &&
               root[i]["channel"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        uint8_t channel = (uint8_t)static_cast<int>(root[i]["channel"]);

        uint8_t calculatedAddress = (nucleo << 4) | channel;
        controllersByName[name] =
                new Controller(name, calculatedAddress, motorMaxVoltage, driverVoltage);

        if (root[i].hasMember("quadCPR") &&
            root[i]["quadCPR"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->quadCPR = (float)static_cast<double>(root[i]["quadCPR"]);
        }
        if (root[i].hasMember("kP") &&
            root[i]["kP"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kP = (float)static_cast<double>(root[i]["kP"]);
        }
        if (root[i].hasMember("kI") &&
            root[i]["kI"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kI = (float)static_cast<double>(root[i]["kI"]);
        }
        if (root[i].hasMember("kD") &&
            root[i]["kD"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->kD = (float)static_cast<double>(root[i]["kD"]);
        }
        if (root[i].hasMember("inversion") &&
            root[i]["inversion"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            controllersByName[name]->inversion = (float)static_cast<double>(root[i]["inversion"]);
        }
        printf("Virtual Controller %s on Nucleo %i channel %i \n", name.c_str(), nucleo, channel);
    }
}