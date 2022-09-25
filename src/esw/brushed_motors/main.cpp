#include "ControllerMap.h" // for ControllerMap
#include "I2C.h"           // for I2C
#include "ROSHandler.h"    // for ROSHandler
#include <ros/ros.h>       // for ros and ROS_INFO

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "brushed_motors");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue controllersRoot;
    nh.getParam("brushed_motors/controllers", controllersRoot);

    ControllerMap::init(controllersRoot);
    ROSHandler::init(&nh);

    std::string i2cDeviceFile;
    nh.getParam("brushed_motors/i2c_device_file", i2cDeviceFile);
    I2C::init(i2cDeviceFile);

    ROS_INFO("Initialization Done. Looping. \n");

    ros::Rate loopRate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}