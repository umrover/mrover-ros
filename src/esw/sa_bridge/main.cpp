#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <motors_manager.hpp>

std::unique_ptr<MotorsManager> SAManager;
std::vector<std::string> SANames =
        {"sa_x", "sa_y", "sa_z", "scoop", "drill"};
std::unique_ptr<CANManager> uv_bulb_can_manager;

bool uvBulbCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    std::vector<uint8_t> frame_data = {0};  // TODO
    uv_bulb_can_manager->send_raw_data(frame_data);
    ROS_INFO("TODO - request for UV Bulb on SA,. Value: %s", req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    uv_bulb_can_manager = std::make_unique<CANManager>(nh, "uv_bulb");
    SAManager = std::make_unique<MotorsManager>(nh, "sa", SANames);
    nh.advertiseService("sa_enable_uv_bulb", uvBulbCallback);
    // Enter the ROS event loop
    ros::spin();

    return 0;
}