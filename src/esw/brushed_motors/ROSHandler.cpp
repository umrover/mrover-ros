#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: static variables
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    // Initialize robotic arm (RA)
    RANames = {"joint_a", "joint_b", "joint_f", "finger", "gripper"};

    moveRASubscriber = n->subscribe<sensor_msgs::JointState>("ra_cmd", 1, moveRA);

    jointDataRA.name = std::vector<std::string>(RANames.begin(), RANames.end());
    jointDataRA.position = std::vector<double>(RANames.size(), 0);
    jointDataRA.velocity = std::vector<double>(RANames.size(), std::nan(""));
    jointDataRA.effort = std::vector<double>(RANames.size(), std::nan(""));

    jointDataPublisherRA = n->advertise<sensor_msgs::JointState>("ra_data", 1);

    // Initialize sample acquisition (SA)
    SANames = {"sa_joint_1", "sa_joint_2", "sa_joint_3", "sa_joint_4", "scoop", "microscope"};

    moveSASubscriber = n->subscribe<sensor_msgs::JointState>("sa_cmd", 1, moveSA);

    jointDataSA.name = std::vector<std::string>(SANames.begin(), SANames.end());
    jointDataSA.position = std::vector<double>(SANames.size(), 0);
    jointDataSA.velocity = std::vector<double>(SANames.size(), std::nan(""));
    jointDataSA.effort = std::vector<double>(SANames.size(), std::nan(""));

    jointDataPublisherSA = n->advertise<sensor_msgs::JointState>("sa_data", 1);

    // Initialize cache
    moveCacheSubscriber = n->subscribe<sensor_msgs::JointState>("cache_cmd", 1, moveCache);

    // Initialize carousel
    moveCarouselSubscriber = n->subscribe<mrover::Carousel>("carousel_cmd", 1, moveCarousel);

    // Initialize mast gimbal
    moveMastGimbalSubscriber = n->subscribe<mrover::MastGimbal>("mast_gimbal_cmd", 1, moveMastGimbal);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the RA joints in open loop
// and publishes angle data right after.
void ROSHandler::moveRA(const sensor_msgs::JointState::ConstPtr& msg) {

    for (size_t i = 0; i < RANames.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(RANames[i]);
        if(controller_iter != ControllerMap::controllersByName.end()) {
            auto controller = controller_iter->second;

            controller->moveOpenLoop((float) msg->velocity[i]);
            jointDataRA.position[i] = controller->getCurrentAngle();
        }
    }

    jointDataPublisherRA.publish(jointDataRA);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the SA joints in open loop
// and publishes angle data right after.
void ROSHandler::moveSA(const sensor_msgs::JointState::ConstPtr& msg) {

    for (size_t i = 0; i < SANames.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(SANames[i]);
        if(controller_iter != ControllerMap::controllersByName.end()) {
            auto controller = controller_iter->second;

            controller->moveOpenLoop((float) msg->velocity[i]);
            jointDataSA.position[i] = controller->getCurrentAngle();
        }
    }

    jointDataPublisherSA.publish(jointDataSA);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the cache in open loop
void ROSHandler::moveCache(const sensor_msgs::JointState::ConstPtr& msg) {
    auto controller_iter = ControllerMap::controllersByName.find("cache");
    if (controller_iter != ControllerMap::controllersByName.end()) {
        auto name = controller_iter->first;
        auto controller = controller_iter->second;

        if (name == "cache") {
            controller->moveOpenLoop((float) msg->velocity[0]);
        }
        else {
            ROS_ERROR("Expected name of JointState msg to be cache.");
        }
    }
    else {
        ROS_ERROR("Controller for cache is not found in ControllerMap and/or config/esw.yaml.");
    }
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the carousel in either open loop or closed loop depending on the msg
void ROSHandler::moveCarousel(const mrover::Carousel::ConstPtr& msg) {
    auto controller_iter = ControllerMap::controllersByName.find("carousel");
    if (controller_iter != ControllerMap::controllersByName.end()) {
        auto name = controller_iter->first;
        auto controller = controller_iter->second;

        if (name == "carousel") {
            if (msg->open_loop) {
                controller->moveOpenLoop((float) msg->vel);
            }
            else {
                ROS_ERROR("Closed loop is currently not supported for %s", name.c_str());
            }
        }
        else {
            ROS_ERROR("Expected name of JointState msg to be carousel.");
        }
    }
    else {
        ROS_ERROR("Controller for carousel is not found in ControllerMap and/or config/esw.yaml.");
    }

}


// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a mast gimbal.
void ROSHandler::moveMastGimbal(const mrover::MastGimbal::ConstPtr& msg) {
    ControllerMap::controllersByName["mast_gimbal_up_down"]->moveOpenLoop((float) msg->up_down);
    ControllerMap::controllersByName["mast_gimbal_left_right"]->moveOpenLoop((float) msg->left_right);
}
