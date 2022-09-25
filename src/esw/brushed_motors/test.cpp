#include "Controller.h"
#include "ControllerMap.h"
#include "I2C.h"
#include "ROSHandler.h" // for ROSHandler
#include <chrono>       // for chrono
#include <ros/ros.h>    // for ros and ROS_INFO
#include <thread>       // for this_thread and sleep_for
#include <vector>       // for vector

const float TARGET_ANGLE = 45.0f;
const float OPEN_LOOP_INPUT = 6.0f;
const unsigned int SLEEP_MS = 50;

// Just runs a test per motor
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "brushed_motors_test");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue controllersRoot;
    nh.getParam("brushed_motors/controllers", controllersRoot);

    ControllerMap::init(controllersRoot);

    for (auto it: ControllerMap::controllersByName) {
        testOpenLoop(it.second);
    }
}

// Helper sleep function
void sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Test function movement types, with focus on open loop
void testOpenLoop(Controller* controller) {
    std::vector<float> openLoopSpeeds = {-1.0f, 0.0f, 1.0f, 0.0f};
    const int timePerAction = 2000;


    for (auto speed: openLoopSpeeds) {
        for (int i = 0; i < timePerAction / SLEEP_MS; ++i) {
            controller->moveOpenLoop(speed);
            sleep(SLEEP_MS);
        }
    }
}