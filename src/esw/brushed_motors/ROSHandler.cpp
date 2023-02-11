#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: static variables
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    RANames = {"joint_a", "joint_b", "joint_c", "joint_d", "joint_e", "joint_f", "finger", "gripper"};

    openLoopSubscriberRA = n->subscribe<sensor_msgs::JointState>(
            "ra_cmd",
            1,
            moveOpenLoopRACommand);

    openLoopSubscriberMast = n->subscribe<mrover::GimbalCmd>("gimbal_cmd", 1, moveGimbal);

    jointData.name = RANames;
    jointData.position = std::vector<double>(RANames.size(), 0);
    jointData.velocity = std::vector<double>(RANames.size(), std::nan(""));
    jointData.effort = std::vector<double>(RANames.size(), std::nan(""));

    jointDataPublisherRA = n->advertise<sensor_msgs::JointState>("ra_data", 1);
}

// REQUIRES: name is a valid name
// MODIFIES: nothing
// EFFECTS: Moves the RA joints in open loop
// and publishes angle data right after.
void ROSHandler::moveOpenLoopRACommand(const sensor_msgs::JointState::ConstPtr& msg) {

    for (size_t i = 0; i < RANames.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(RANames[i]);
        if(controller_iter != ControllerMap::controllersByName.end()) {
            auto name = controller_iter->first;
            auto controller = controller_iter->second;

            controller->moveOpenLoop((float) msg->velocity[i]);
            jointData.position[i] = controller->getCurrentAngle();
            // TODO - get calibration status
        }
    }

    // TODO - publish calibration status
    jointDataPublisherRA.publish(jointData);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a gimbal.
void ROSHandler::moveGimbal(const mrover::GimbalCmd::ConstPtr& msg) {
    ControllerMap::controllersByName["mast_up_down"]->moveOpenLoop((float) msg->up_down);
    ControllerMap::controllersByName["mast_left_right"]->moveOpenLoop((float) msg->left_right);
}

// TODO
void processCalibrate(
    mrover::CalibrateMotors::Request &req,
    mrover::CalibrateMotors::Response &res
) {
    // TODO - make sure that req.names.size() == req.calibrate.size()

    for (size_t i = 0; i < req.names.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(req.names[i]);
        auto& [name, controller] = *controller_iter;

        // 1. Check if should calibrate.

        bool shouldCalibrate = true;

        if (controller_iter == ControllerMap::controllersByName.end()) {
            shouldCalibrate = false;
        }
        else if (1) {// (controller_iter->isAlreadyCalibrated) { // TODO - CHANGE
            shouldCalibrate = false;
        }
        else if (1) {// (controller_iter->limitSwitchEnabled) { // TODO - CHANGE
            shouldCalibrate = false;
        }

        // 2. Carry out calibration and update response

        if(shouldCalibrate) {
            float calibrationSpeed = false; // TODO - DO STUFF -> call moveOpenLoop with numbers from yaml
            controller->moveOpenLoop(calibrationSpeed);
            res.actively_calibrating.push_back(true);
        }
        else {
            res.actively_calibrating.push_back(false);    
        }
    }
}
