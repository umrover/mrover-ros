#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: static variables
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    // Initialize robotic arm (RA)
    RANames = {"joint_a", "joint_b", "joint_f", "finger", "gripper"};

    moveRASubscriber = n->subscribe<sensor_msgs::JointState>("ra_cmd", 1, moveRA);

    calibrateService = n->advertiseService<mrover::CalibrateMotors::Request, mrover::CalibrateMotors::Response>("calibrate", processMotorCalibrate);

    jointDataRA.name = std::vector<std::string>(RANames.begin(), RANames.end());
    jointDataRA.position = std::vector<double>(RANames.size(), 0);
    jointDataRA.velocity = std::vector<double>(RANames.size(), std::nan(""));
    jointDataRA.effort = std::vector<double>(RANames.size(), std::nan(""));

    calibrationStatusRA.names = std::vector<std::string>(RANames.begin(), RANames.end());
    calibrationStatusRA.calibrated = std::vector<uint8_t>(calibrationStatusRA.names.size(), false);
    calibrationStatusPublisherRA = n->advertise<mrover::Calibrated>("ra_is_calibrated", 1);
    jointDataPublisherRA = n->advertise<sensor_msgs::JointState>("ra_data", 1);

    // Initialize sample acquisition (SA)
    SANames = {"sa_joint_1", "sa_joint_2", "sa_joint_3", "scoop", "microscope"};

    moveSASubscriber = n->subscribe<sensor_msgs::JointState>("sa_cmd", 1, moveSA);

    jointDataSA.name = std::vector<std::string>(SANames.begin(), SANames.end());
    jointDataSA.position = std::vector<double>(SANames.size(), 0);
    jointDataSA.velocity = std::vector<double>(SANames.size(), std::nan(""));
    jointDataSA.effort = std::vector<double>(SANames.size(), std::nan(""));

    calibrationStatusSA.names = std::vector<std::string>(SANames.begin(), SANames.end());
    calibrationStatusSA.calibrated = std::vector<uint8_t>(calibrationStatusSA.names.size(), false);
    calibrationStatusPublisherSA = n->advertise<mrover::Calibrated>("sa_is_calibrated", 1);
    jointDataPublisherSA = n->advertise<sensor_msgs::JointState>("sa_data", 1);

    // Initialize cache
    moveCacheSubscriber = n->subscribe<sensor_msgs::JointState>("cache_cmd", 1, moveCache);

    // Initialize carousel
    calibrationStatusCarousel.names = {"carousel"};
    calibrationStatusCarousel.calibrated = {false};
    calibrationStatusPublisherCarousel = n->advertise<mrover::Calibrated>("carousel_is_calibrated", 1);
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
        ROS_ERROR("Could not find controller named %s.", name.c_str());
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
    for (size_t i = 0; i < RANames.size(); ++i) {
        std::optional<float> pos = moveControllerOpenLoop(RANames[i], (float) msg->velocity[i]);
        jointDataRA.position[i] = pos.value_or(0.0);

        std::optional<bool> calibrated = getControllerCalibrated(RANames[i]);
        calibrationStatusRA.calibrated[i] = calibrated.value_or(false);
    }
    calibrationStatusPublisherRA.publish(calibrationStatusRA);
    jointDataPublisherRA.publish(jointDataRA);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Determine if a controller is calibrated
std::optional<bool> ROSHandler::getControllerCalibrated(const std::string& name) {
    auto controller_iter = ControllerMap::controllersByName.find(name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        ROS_ERROR("Could not find controller named %s.", name.c_str());
        return std::nullopt;
    }

    Controller* controller = controller_iter->second;

    return std::make_optional<bool>(controller->getCalibrationStatus());
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves the SA joints in open loop and publishes angle data right after.
void ROSHandler::moveSA(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < SANames.size(); ++i) {
        std::optional<float> pos = moveControllerOpenLoop(SANames[i], (float) msg->velocity[i]);
        jointDataSA.position[i] = pos.value_or(0.0);

        std::optional<bool> calibrated = getControllerCalibrated(SANames[i]);
        calibrationStatusSA.calibrated[i] = calibrated.value_or(false);
    }
    calibrationStatusPublisherSA.publish(calibrationStatusSA);
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
    } else {
        ROS_ERROR("Closed loop is currently not supported for carousel commands.");
    }
    std::optional<bool> calibrated = getControllerCalibrated("carousel");
    calibrationStatusCarousel.calibrated[0] = calibrated.value_or(false);
    calibrationStatusPublisherCarousel.publish(calibrationStatusCarousel);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a mast gimbal in open loop.
void ROSHandler::moveMastGimbal(const mrover::MastGimbal::ConstPtr& msg) {
    moveControllerOpenLoop("mast_gimbal_up_down", (float) msg->up_down);
    moveControllerOpenLoop("mast_gimbal_left_right", (float) msg->left_right);
}

// REQUIRES: valid req and res objects
// MODIFIES: res
// EFFECTS: sends a move/calibration command to the mcu

bool ROSHandler::processMotorCalibrate(mrover::CalibrateMotors::Request& req, mrover::CalibrateMotors::Response& res) {
    assert(req.names.size() == req.calibrate.size());

    for (size_t i = 0; i < req.names.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(req.names[i]);
        auto& [name, controller] = *controller_iter;

        // Check if already calibrated
        controller->askIsCalibrated();

        // Determine if calibration is needed
        bool shouldCalibrate = !(controller_iter == ControllerMap::controllersByName.end() || controller->getCalibrationStatus() || !controller->getLimitSwitchEnabled());

        // Calibrate
        if (shouldCalibrate) {

            controller->moveOpenLoop(controller->calibrationSpeed);
            controller->askIsCalibrated();

            res.actively_calibrating.push_back(true);
        } else {
            res.actively_calibrating.push_back(false);
        }
    }

    return true;
}
