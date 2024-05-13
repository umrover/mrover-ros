#include <ros/init.h>
#include <ros/node_handle.h>

#include <motors_group.hpp>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "simple_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh{"~"};

    auto groupName = pnh.param<std::string>("group_name", {});
    if (groupName.empty()) throw std::runtime_error{"Group name is reuqired for a simple bridge!"};

    mrover::MotorsGroup group{nh, groupName};

    ros::spin();

    return EXIT_SUCCESS;
}
