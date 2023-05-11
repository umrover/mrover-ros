#include "ControllerMap.h" // for ControllerMap
#include "I2C.h"           // for I2C
#include "ROSHandler.h"    // for ROSHandler
#include "Test.h"          // for mainTest
#include "UART.h"          // for UART
#include <ros/ros.h>       // for ros and ROS_INFO

int main(int argc, char** argv) {

    ros::init(argc, argv, "brushed_motors");
    ros::NodeHandle nh;

    bool isTest;
    nh.getParam("brushed_motors/test", isTest);

    XmlRpc::XmlRpcValue controllersRoot;
    nh.getParam("brushed_motors/controllers", controllersRoot);

    ControllerMap::init(controllersRoot);

    bool use_uart_and_send_only; // as opposed to i2c
    nh.getParam("brushed_motors/use_uart_and_send_only", use_uart_and_send_only);

    if (use_uart_and_send_only) {
        std::string uartDeviceFile;
        nh.getParam("brushed_motors/uart_device_file", uartDeviceFile);
        UART::init(uartDeviceFile);
    } else {
        std::string i2cDeviceFile;
        nh.getParam("brushed_motors/i2c_device_file", i2cDeviceFile);
        I2C::init(i2cDeviceFile);
    }

    if (isTest) {
        if (use_uart_and_send_only) {
            for (auto& [name, controller]: ControllerMap::controllersByName) {
                ROS_INFO("Conducting tests on %s \n", name.c_str());
                Test::testOpenLoopViaUART(controller);
            }
        } else {
            for (auto& [name, controller]: ControllerMap::controllersByName) {
                ROS_INFO("Conducting tests on %s \n", name.c_str());
                Test::testOpenLoop(controller);
                bool isCalibrated = Test::testCalibrated(controller);
                ROS_INFO("Calibration status on %s is %d\n", name.c_str(), isCalibrated);
            }
        }
    } else {
        ROSHandler::init(&nh, use_uart_and_send_only);

        ROS_INFO("Initialization Done. \nLooping. \n");

        // Refresh every 0.3 seconds. 0.3s is arbitrary, but as long as it is less than 443ms (watchdog of STM32 MCU).
        ros::Timer timer = nh.createTimer(ros::Duration(0.3), ROSHandler::timerCallback);
        ros::spin();
    }

    return 0;
}