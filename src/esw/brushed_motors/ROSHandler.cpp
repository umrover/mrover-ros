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

    calibrationStatusRA.names = RANames;
    // Default to no motors calibrated
    for (auto name : calibrationStatusRA.names) {
        calibrationStatusRA.calibrated.push_back(false);
    }

    calibrationStatusPublisherRA = n->advertise<mrover::Calibrated>("ra_is_calibrated", 1);
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
            auto& [name, controller] = *controller_iter;

            controller->moveOpenLoop((float) msg->velocity[i]);
            jointData.position[i] = controller->getCurrentAngle();
            calibrationStatusRA.calibrated[i] = controller->getCalibrationStatus();
        }
    }

    calibrationStatusPublisherRA.publish(calibrationStatusRA);
    jointDataPublisherRA.publish(jointData);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a gimbal.
void ROSHandler::moveGimbal(const mrover::GimbalCmd::ConstPtr& msg) {
    ControllerMap::controllersByName["mast_up_down"]->moveOpenLoop((float) msg->up_down);
    ControllerMap::controllersByName["mast_left_right"]->moveOpenLoop((float) msg->left_right);
}

// REQUIRES: valid req and res objects
// MODIFIES: res
// EFFECTS: sends a move/calibration command to the mcu
void processCalibrate(
    mrover::CalibrateMotors::Request &req,
    mrover::CalibrateMotors::Response &res
) {
    assert(req.names.size() == req.calibrate.size());

    for (size_t i = 0; i < req.names.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(req.names[i]);
        auto& [name, controller] = *controller_iter;

        // Determine if calibration is needed
        bool shouldCalibrate = !(controller_iter == ControllerMap::controllersByName.end()
                                 || controller->getCalibrationStatus()
                                 || !controller->getLimitSwitchEnabled());

        // Calibrate
        if(shouldCalibrate) {
            controller->moveOpenLoop(controller->calibrationSpeed);
            controller->askIsCalibrated();
            res.actively_calibrating.push_back(true);
        } else {
            res.actively_calibrating.push_back(false);    
        }
    }
}
