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
    SANames = {"sa_joint_1", "sa_joint_2", "sa_joint_3", "scoop", "microscope"};

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
// EFFECTS: Moves a controller in open loop.
std::optional<float> ROSHandler::moveControllerOpenLoop(const std::string& name, float velocity) {
    auto controller_iter = ControllerMap::controllersByName.find(name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        // ROS_ERROR("Could not find controller named %s.", name.c_str());
        return std::nullopt;
    }

    Controller* controller = controller_iter->second;
    controller->moveOpenLoop(velocity);

    return std::make_optional<float>(controller->getCurrentAngle());
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the RA joints in open loop and publishes angle data right after.
// Note: any invalid controllers will be published with a position of 0.
void ROSHandler::moveRA(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::optional<float> pos = moveControllerOpenLoop(msg->name[i], (float) msg->velocity[i]);
        jointDataRA.position[i] = pos.value_or(0.0);
    }

    jointDataPublisherRA.publish(jointDataRA);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the SA joints in open loop and publishes angle data right after.
void ROSHandler::moveSA(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < SANames.size(); ++i) {
        std::optional<float> pos = moveControllerOpenLoop(SANames[i], (float) msg->velocity[i]);
        jointDataSA.position[i] = pos.value_or(0.0);
    }
    jointDataPublisherSA.publish(jointDataSA);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the cache in open loop.
void ROSHandler::moveCache(const sensor_msgs::JointState::ConstPtr& msg) {
    moveControllerOpenLoop("cache", (float) msg->velocity[0]);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the carousel in either open loop or closed loop depending on msg.
void ROSHandler::moveCarousel(const mrover::Carousel::ConstPtr& msg) {
    if (msg->open_loop) {
        moveControllerOpenLoop("carousel", (float) msg->vel);
    }
    else {
        ROS_ERROR("Closed loop is currently not supported for carousel commands.");
    }

}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a mast gimbal in open loop.
void ROSHandler::moveMastGimbal(const mrover::MastGimbal::ConstPtr& msg) {
    moveControllerOpenLoop("mast_gimbal_up_down", (float) msg->up_down);
    moveControllerOpenLoop("mast_gimbal_left_right", (float) msg->left_right);
}
