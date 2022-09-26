#include "Test.h"

const float TARGET_ANGLE = 45.0f;
const float OPEN_LOOP_INPUT = 6.0f;
const unsigned int SLEEP_MS = 50;

// Helper sleep function
void Test::sleepHelper(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Test function movement types, with focus on open loop
void Test::testOpenLoop(Controller* controller) {
    std::vector<float> openLoopSpeeds = {-1.0f, 0.0f, 1.0f, 0.0f};
    const int timePerAction = 2000;


    for (auto speed: openLoopSpeeds) {
        for (int i = 0; i < (int) (timePerAction / SLEEP_MS); ++i) {
            controller->moveOpenLoop(speed);
            sleepHelper(SLEEP_MS);
        }
    }
}

// Just runs a test per motor
Test::Test(ros::NodeHandle* nh) {
    XmlRpc::XmlRpcValue controllersRoot;
    nh->getParam("brushed_motors/controllers", controllersRoot);

    ControllerMap::init(controllersRoot);

    std::string i2cDeviceFile;
    nh->getParam("brushed_motors/i2c_device_file", i2cDeviceFile);
    I2C::init(i2cDeviceFile);
}