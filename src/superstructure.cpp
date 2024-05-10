#include <ros/init.h>
#include <ros/node_handle.h>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "superstructure");
    ros::NodeHandle nh;

    return 0;
}