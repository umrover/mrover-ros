#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include "Controller.h"
#include "I2C.h"
#include "ROSHandler.h"

//Handles instantiation of Controller objects, FrontEnd, and BackEnd classes

//The outgoing function calls on the ROSHandler's handle_outgoing() function every millisecond
void outgoing() {
    while (true) {
        ROSHandler::handle_outgoing();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

//The incoming function calls on the ROSHandler's handle_incoming() function continuously
void incoming() {
    while (true) {
        ROSHandler::handle_incoming();
    }
}

int main() {
    ros::init(argc, argv, "motors");

    printf("Initializing virtual controllers\n");
    ControllerMap::init();

    printf("Initializing ROS bus\n");
    ROSHandler::init();

    printf("Initializing I2C bus\n");
    I2C::init();

    printf("Initialization Done. Looping. Reduced output for program speed.\n");
    std::thread outThread(&outgoing);
    std::thread inThread(&incoming);

    outThread.join();
    inThread.join();

    return 0;
}
