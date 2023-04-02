#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: static variables
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    // Initialize services
    calibrateService = n->advertiseService<mrover::CalibrateMotors::Request, mrover::CalibrateMotors::Response>("calibrate", processMotorCalibrate);
    adjustService = n->advertiseService<mrover::AdjustMotors::Request, mrover::AdjustMotors::Response>("adjust", processMotorAdjust);
    adjustUsingAbsEncService = n->advertiseService<mrover::AdjustMotors::Request, mrover::AdjustMotors::Response>("adjust_using_abs_enc", processMotorAdjustUsingAbsEnc);
    enableLimitSwitchService = n->advertiseService<mrover::EnableDevice::Request, mrover::EnableDevice::Response>("enable_limit_switch", processMotorEnableLimitSwitches);

    // Initialize robotic arm (RA)
    RANames = {"joint_a", "joint_b", "joint_f", "finger", "gripper"};

    moveRASubscriber = n->subscribe<sensor_msgs::JointState>("ra_cmd", 1, moveRA);

    jointDataRA.name = std::vector<std::string>(RANames.begin(), RANames.end());
    jointDataRA.position = std::vector<double>(RANames.size(), 0);
    jointDataRA.velocity = std::vector<double>(RANames.size(), 0);
    jointDataRA.effort = std::vector<double>(RANames.size(), 0);

    calibrationStatusRA.names = std::vector<std::string>(RANames.begin(), RANames.end());
    calibrationStatusRA.calibrated = std::vector<uint8_t>(calibrationStatusRA.names.size(), false);
    calibrationStatusPublisherRA = n->advertise<mrover::Calibrated>("ra_is_calibrated", 1);
    jointDataPublisherRA = n->advertise<sensor_msgs::JointState>("brushed_ra_data", 1);

    // Initialize sample acquisition (SA)
    SANames = {"sa_joint_1", "sa_joint_2", "sa_joint_3", "scoop", "microscope"};

    moveSASubscriber = n->subscribe<sensor_msgs::JointState>("sa_cmd", 1, moveSA);

    jointDataSA.name = std::vector<std::string>(SANames.begin(), SANames.end());
    jointDataSA.position = std::vector<double>(SANames.size(), 0);
    jointDataSA.velocity = std::vector<double>(SANames.size(), 0);
    jointDataSA.effort = std::vector<double>(SANames.size(), 0);

    calibrationStatusSA.names = std::vector<std::string>(SANames.begin(), SANames.end());
    calibrationStatusSA.calibrated = std::vector<uint8_t>(calibrationStatusSA.names.size(), false);
    calibrationStatusPublisherSA = n->advertise<mrover::Calibrated>("sa_is_calibrated", 1);
    jointDataPublisherSA = n->advertise<sensor_msgs::JointState>("sa_data", 1);

    // Initialize cache
    moveCacheSubscriber = n->subscribe<sensor_msgs::JointState>("cache_cmd", 1, moveCache);

    // Initialize carousel
    carousel_name = "carousel";
    calibrationStatusCarousel.names = {carousel_name};
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
    int mappedIndex = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if ((i == 2) || (i == 3) || (i == 4)) {
            // We expect msg->name to be joints a, b, c, d, e, f, finger, and gripper.
            // Skip if msg->name[i] is joints c, d, or e. So skip if i == 2, 3, or 4.
            continue;
        }
        std::optional<float> pos = moveControllerOpenLoop(msg->name[i], (float) msg->velocity[i]);

        jointDataRA.position[mappedIndex] = pos.value_or(0.0);

        std::optional<bool> calibrated = getControllerCalibrated(msg->name[i]);
        calibrationStatusRA.calibrated[mappedIndex] = calibrated.value_or(false);
        ++mappedIndex;
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
    return std::make_optional<bool>(controller->isCalibrated());
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
    auto controller_iter = ControllerMap::controllersByName.find(req.name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        ROS_ERROR("Could not find controller named %s.", req.name.c_str());
        res.actively_calibrating = false;
        return true;
    }

    auto& [name, controller] = *controller_iter;

    // Determine if calibration is needed
    bool isCalibrated = controller->isCalibrated();
    publish_calibration_data_using_name(req.name, isCalibrated);

    bool shouldCalibrate = !isCalibrated && controller->getLimitSwitchEnabled();

    // Calibrate
    if (shouldCalibrate) {
        controller->moveOpenLoop(controller->calibrationVel);
        res.actively_calibrating = true;
    } else {
        res.actively_calibrating = false;
    }

    return true;
}

// REQUIRES: valid req and res objects
// MODIFIES: res
// EFFECTS: hard sets the requested controller angle
bool ROSHandler::processMotorAdjust(mrover::AdjustMotors::Request& req, mrover::AdjustMotors::Response& res) {

    auto controller_iter = ControllerMap::controllersByName.find(req.name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        ROS_ERROR("Could not find controller named %s.", req.name.c_str());
        res.success = false;
        return true;
    }

    auto& [name, controller] = *controller_iter;
    controller->overrideCurrentAngle(req.value);
    publish_calibration_data_using_name(req.name, controller->isCalibrated());

    res.success = true;
    res.abs_enc_rad = controller->getAbsoluteEncoderValue();

    return true;
}

// REQUIRES: valid req and res objects
// MODIFIES: res
// EFFECTS: takes the current absolute encoder value, applies an offset, and hard sets the new angle
bool ROSHandler::processMotorAdjustUsingAbsEnc(mrover::AdjustMotors::Request& req, mrover::AdjustMotors::Response& res) {

    auto controller_iter = ControllerMap::controllersByName.find(req.name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        ROS_ERROR("Could not find controller named %s.", req.name.c_str());
        res.success = false;
        return true;
    }

    auto& [name, controller] = *controller_iter;
    float abs_enc_value = controller->getAbsoluteEncoderValue();
    float new_value = req.value - abs_enc_value;
    controller->overrideCurrentAngle(new_value);
    publish_calibration_data_using_name(req.name, controller->isCalibrated());

    res.success = true;
    res.abs_enc_rad = abs_enc_value;

    return true;
}

// REQUIRES: valid req and res objects
// MODIFIES: res
// EFFECTS: disables or enables limit switches
bool ROSHandler::processMotorEnableLimitSwitches(mrover::EnableDevice::Request& req, mrover::EnableDevice::Response& res) {

    auto controller_iter = ControllerMap::controllersByName.find(req.name);

    if (controller_iter == ControllerMap::controllersByName.end()) {
        ROS_ERROR("Could not find controller named %s.", req.name.c_str());
        res.success = false;
        return true;
    }

    auto& [name, controller] = *controller_iter;
    controller->enableLimitSwitches(req.enable);
    res.success = true;

    return true;
}

// REQUIRES: name is the name of a controller and isCalibrated is whether it is calibrated
// MODIFIES: static variables
// EFFECTS: Publishes calibration status to the proper topic depending on the name
void ROSHandler::publish_calibration_data_using_name(const std::string& name, bool isCalibrated) {
    auto ra_iter = std::find(RANames.begin(), RANames.end(), name);
    if (ra_iter != RANames.end()) {
        std::size_t ra_idx = std::distance(RANames.begin(), ra_iter);
        calibrationStatusRA.calibrated[ra_idx] = isCalibrated;
        calibrationStatusPublisherRA.publish(calibrationStatusRA);
    }
    else if (carousel_name == name) {
        calibrationStatusCarousel.calibrated[0] = isCalibrated;
        calibrationStatusPublisherCarousel.publish(calibrationStatusCarousel);
    }
    else {
        auto sa_iter = std::find(SANames.begin(), SANames.end(), name);
        if (sa_iter != SANames.end()) {
            std::size_t sa_idx = std::distance(SANames.begin(), sa_iter);
            calibrationStatusSA.calibrated[sa_idx] = isCalibrated;
            calibrationStatusPublisherSA.publish(calibrationStatusSA);
        }
    }
}