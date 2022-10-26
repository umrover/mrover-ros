#include "ControllerMap.h"
#include "I2C.h"
#include <chrono>       // for chrono
#include <ros/ros.h>    // for ros and ROS_INFO
#include <thread>       // for this_thread and sleep_for
#include <vector>       // for vector

void mainTest(ros::NodeHandle* nh);

class Test {
public:

    // Test function movement types, with focus on open loop
    static void testOpenLoop(Controller* controller);

private:
    // Helper sleep function
    static void sleepHelper(int ms);

    static const unsigned int SLEEP_MS = 50;
};