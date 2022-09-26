#include "ControllerMap.h"
#include "I2C.h"
#include <chrono>       // for chrono
#include <ros/ros.h>    // for ros and ROS_INFO
#include <thread>       // for this_thread and sleep_for
#include <vector>       // for vector

void mainTest(ros::NodeHandle* nh);

class Test {
public:
    // Just runs a test per motor
    Test(ros::NodeHandle* nh);

    // Test function movement types, with focus on open loop
    void testOpenLoop(Controller* controller);

private:
    // Helper sleep function
    void sleepHelper(int ms);

    float TARGET_ANGLE;
    float OPEN_LOOP_INPUT;
    unsigned int SLEEP_MS;
};