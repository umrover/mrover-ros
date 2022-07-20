#include "ROSHandler.h"

// Initialize the ROS bus and subscribe to relevant channels with message handlers defined below
void ROSHandler::init() {
    // Creation of ROS bus
    &n = new ros::NodeHandle();

    if (!ros::ok()) {
        printf("ROS Node not created\n");
        exit(1);
    } else {
        printf("ROS Node created\n");
    }

    internal_object = new InternalHandler();

    // Subscription to ROS channels
    *enable_scoop_limit_switch_sub = n->subscribe("enable_scoop_limit_switch_cmd", 1, ROSHandler::InternalHandler::enable_scoop_limit_switch_cmd);
    *move_arm_closed_loop_sub = n->subscribe("arm_closed_loop_cmd", 1, ROSHandler::InternalHandler::move_arm_closed_loop_cmd);
    *move_arm_open_loop_sub = n->subscribe("move_arm_open_loop_cmd", 1, ROSHandler::InternalHandler::move_arm_open_loop_cmd);
    *move_carousel_closed_loop_sub = n->subscribe("move_carousel_closed_loop_cmd", 1, ROSHandler::InternalHandler::move_carousel_closed_loop_cmd);
    *move_carousel_open_loop_sub = n->subscribe("move_carousel_open_loop_cmd", 1, ROSHandler::InternalHandler::move_carousel_open_loop_cmd);
    *move_hand_open_loop_sub = n->subscribe("move_hand_open_loop_cmd", 1, ROSHandler::InternalHandler::move_hand_open_loop_cmd);
    *move_mast_gimbal_sub = n->subscribe("move_mast_gimbal_cmd", 1, ROSHandler::InternalHandler::move_mast_gimbal_cmd);
    *move_sa_closed_loop_sub = n->subscribe("sa_closed_loop_cmd", 1, ROSHandler::InternalHandler::move_sa_closed_loop_cmd);
    *move_sa_open_loop_sub = n->subscribe("move_sa_open_loop_cmd", 1, ROSHandler::InternalHandler::move_sa_open_loop_cmd);
    *move_science_hand_open_loop_sub = n->subscribe("move_science_hand_open_loop_cmd", 1, ROSHandler::InternalHandler::move_science_hand_open_loop_cmd);
    *zero_carousel_sub = n->subscribe("zero_carousel_cmd", 1, ROSHandler::InternalHandler::zero_carousel_cmd);

    *arm_b_calib_data_pub = n->advertise<mrover::Calibrate>("arm_b_calib_data", 1);
    *arm_position_pub = n->advertise<mrover::ArmPosition>("arm_position", 1);
    *carousel_calib_data_pub = n->advertise<mrover::Calibrate>("carousel_calib_data", 1);
    *carousel_pos_data_pub = n->advertise<mrover::CarouselPosition>("carousel_pos_data", 1);
    *sa_b_calib_data_pub = n->advertise<mrover::Calibrate>("sa_b_calib_data", 1);
    *sa_position_pub = n->advertise<mrover::SAPosition>("sa_position", 1);

    ros::Rate loop_rate(10);
}

// Handles a single incoming ROS message
void ROSHandler::handle_incoming() {
    ros::spinOnce();
}

// Decide whether outgoing messages need to be sent, and if so, send them
void ROSHandler::handle_outgoing() {
    // If the last time arm position messages were outputted was over 200 ms ago, get new data from Controllers to be sent
    std::chrono::duration heartbeat_dead_time = std::chrono::milliseconds(120);

    // This is used as a heart beat (to make sure that nucleos do not reset, but honestly that's too difficult)
    if (NOW - last_heartbeat_output_time > heartbeat_dead_time) {
        internal_object->refresh_arm_quad_angles();
        internal_object->refresh_sa_quad_angles();
        internal_object->refresh_carousel_quad_angles();
    }


    std::chrono::duration calib_data_output_dead_time = std::chrono::milliseconds(1000);
    // Refresh and post joint b calibration data every second
    if (NOW - last_calib_data_output_time > calib_data_output_dead_time) {
        internal_object->refresh_carousel_calib_data();
        internal_object->refresh_arm_calib_data();
        internal_object->refresh_sa_calib_data();
    }
}

void ROSHandler::InternalHandler::enable_scoop_limit_switch_cmd(mrover::Enable& msg) {
    ControllerMap::controllers["SCIENCE_HAND_SCOOP"]->enable_limit_switch(msg->enable);
}

void ROSHandler::InternalHandler::move_arm_closed_loop_cmd(mrover::ArmPosition& msg) {
    ControllerMap::controllers["ARM_A"]->move_closed_loop(0.0f, msg->joints[0]);
    ControllerMap::controllers["ARM_B"]->move_closed_loop(0.0f, msg->joints[1]);
    ControllerMap::controllers["ARM_C"]->move_closed_loop(0.0f, msg->joints[2]);
    ControllerMap::controllers["ARM_D"]->move_closed_loop(0.0f, msg->joints[3]);
    ControllerMap::controllers["ARM_E"]->move_closed_loop(0.0f, msg->joints[4]);
    ControllerMap::controllers["ARM_F"]->move_closed_loop(0.0f, msg->joints[5]);
    publish_arm_pos_data();
}

void ROSHandler::InternalHandler::move_arm_open_loop_cmd(mrover::ArmOpenLoopCmd& msg) {
    ControllerMap::controllers["ARM_A"]->move_open_loop(msg->throttle[0]);
    ControllerMap::controllers["ARM_B"]->move_open_loop(msg->throttle[1]);
    ControllerMap::controllers["ARM_C"]->move_open_loop(msg->throttle[2]);
    ControllerMap::controllers["ARM_D"]->move_open_loop(msg->throttle[3]);
    ControllerMap::controllers["ARM_E"]->move_open_loop(msg->throttle[4]);
    ControllerMap::controllers["ARM_F"]->move_open_loop(msg->throttle[5]);

    publish_arm_pos_data();
}

void ROSHandler::InternalHandler::move_carousel_closed_loop_cmd(mrover::CarouselPosition& msg) {
    ControllerMap::controllers["CAROUSEL_MOTOR"]->move_closed_loop(0.0f, msg->position);
}

void ROSHandler::InternalHandler::move_carousel_open_loop_cmd(mrover::CarouselOpenLoopCmd& msg) {
    ControllerMap::controllers["CAROUSEL_MOTOR"]->move_open_loop(msg->throttle);
}

void ROSHandler::InternalHandler::move_hand_open_loop_cmd(mrover::HandCmd& msg) {
    ControllerMap::controllers["HAND_FINGER"]->move_open_loop(msg->finger);
    ControllerMap::controllers["HAND_GRIP"]->move_open_loop(msg->grip);
}

void ROSHandler::InternalHandler::move_mast_gimbal_cmd(mrover::MastGimbalCmd& msg) {
    ControllerMap::controllers["GIMBAL_PITCH"]->move_open_loop(msg->pitch[0]);
    ControllerMap::controllers["GIMBAL_YAW"]->move_open_loop(msg->yaw[0]);
}

void ROSHandler::InternalHandler::move_sa_closed_loop_cmd(mrover::SAPosition& msg) {
    ControllerMap::controllers["SA_A"]->move_closed_loop(0.0f, msg->joint_a);
    ControllerMap::controllers["SA_B"]->move_closed_loop(0.0f, msg->joint_b);
    ControllerMap::controllers["SA_C"]->move_closed_loop(0.0f, msg->joint_c);
    ControllerMap::controllers["SA_E"]->move_closed_loop(0.0f, msg->joint_e);

    publish_sa_pos_data();
}

void ROSHandler::InternalHandler::move_sa_open_loop_cmd(mrover::SAOpenLoopCmd& msg) {
    ControllerMap::controllers["SA_A"]->move_open_loop(msg->throttle[0]);
    ControllerMap::controllers["SA_B"]->move_open_loop(msg->throttle[1]);
    ControllerMap::controllers["SA_C"]->move_open_loop(msg->throttle[2]);
    ControllerMap::controllers["SA_E"]->move_open_loop(msg->throttle[3]);

    publish_sa_pos_data();
}

void ROSHandler::InternalHandler::move_science_hand_open_loop_cmd(mrover::ScienceHandCmd& msg) {
    ControllerMap::controllers["SCIENCE_HAND_SENSOR"]->move_open_loop(msg->microscope_triad);
    ControllerMap::controllers["SCIENCE_HAND_SCOOP"]->move_open_loop(msg->scoop);
}

void ROSHandler::InternalHandler::publish_arm_calib_data() {
    mrover::Calibrate msg;
    msg.calibrated = ControllerMap::controllers["ARM_B"]->calibrated;
    arm_b_calib_data_pub.publish(msg);
    last_calib_data_output_time = NOW;
}

void ROSHandler::InternalHandler::publish_arm_pos_data() {
    mrover::ArmPosition msg;
    msg.joints[0] = ControllerMap::controllers["ARM_A"]->get_current_angle();
    msg.joints[1] = ControllerMap::controllers["ARM_B"]->get_current_angle();
    msg.joints[2] = ControllerMap::controllers["ARM_C"]->get_current_angle();
    msg.joints[3] = ControllerMap::controllers["ARM_D"]->get_current_angle();
    msg.joints[4] = ControllerMap::controllers["ARM_E"]->get_current_angle();
    msg.joints[5] = ControllerMap::controllers["ARM_F"]->get_current_angle();
    arm_position_pub.publish(msg);
    last_heartbeat_output_time = NOW;
}

void ROSHandler::InternalHandler::publish_carousel_calib_data() {
    mrover::Calibrate msg;
    msg.calibrated = ControllerMap::controllers["CAROUSEL_MOTOR"]->calibrated;
    carousel_calib_data_pub.publish(msg);
    last_calib_data_output_time = NOW;
}

void ROSHandler::InternalHandler::publish_carousel_pos_data() {
    mrover::CarouselPosition msg;
    float carousel_angle = ControllerMap::controllers["CAROUSEL_MOTOR"]->get_current_angle();
    msg.position_rad = carousel_angle;
    carousel_pos_data_pub.publish(msg);
    last_heartbeat_output_time = NOW;
}

void ROSHandler::InternalHandler::publish_sa_calib_data() {
    mrover::Calibrate msg;
    msg.calibrated = ControllerMap::controllers["SA_B"]->calibrated;
    sa_b_calib_data_pub.publish(msg);
    last_calib_data_output_time = NOW;
}

void ROSHandler::InternalHandler::publish_sa_pos_data() {
    mrover::SAPosition msg;
    msg.joint_a = ControllerMap::controllers["SA_A"]->get_current_angle();
    msg.joint_b = ControllerMap::controllers["SA_B"]->get_current_angle();
    msg.joint_c = ControllerMap::controllers["SA_C"]->get_current_angle();
    msg.joint_e = ControllerMap::controllers["SA_E"]->get_current_angle();
    sa_position_pub.publish(msg);
    last_heartbeat_output_time = NOW;
}

void ROSHandler::InternalHandler::refresh_arm_calib_data() {
    if (!ControllerMap::check_if_live("ARM_B")) {
        return;
    }

    ControllerMap::controllers["ARM_B"]->refresh_calibration_data();
    publish_arm_calib_data();
}

void ROSHandler::InternalHandler::refresh_arm_quad_angles() {
    ControllerMap::controllers["ARM_A"]->refresh_quad_angle();
    ControllerMap::controllers["ARM_B"]->refresh_quad_angle();
    ControllerMap::controllers["ARM_C"]->refresh_quad_angle();
    ControllerMap::controllers["ARM_D"]->refresh_quad_angle();
    ControllerMap::controllers["ARM_E"]->refresh_quad_angle();
    ControllerMap::controllers["ARM_F"]->refresh_quad_angle();

    publish_arm_pos_data();
}

void ROSHandler::InternalHandler::refresh_carousel_calib_data() {
    ControllerMap::controllers["CAROUSEL_MOTOR"]->refresh_calibration_data();
    publish_carousel_calib_data();
}

void ROSHandler::InternalHandler::refresh_carousel_quad_angles() {
    ControllerMap::controllers["CAROUSEL_MOTOR"]->refresh_quad_angle();
    publish_carousel_pos_data();
}

void ROSHandler::InternalHandler::refresh_sa_calib_data() {
    if (!ControllerMap::check_if_live("SA_B")) {
        return;
    }

    ControllerMap::controllers["SA_B"]->refresh_calibration_data();
    publish_sa_calib_data();
}

void ROSHandler::InternalHandler::refresh_sa_quad_angles() {
    ControllerMap::controllers["SA_A"]->refresh_quad_angle();
    ControllerMap::controllers["SA_B"]->refresh_quad_angle();
    ControllerMap::controllers["SA_C"]->refresh_quad_angle();
    ControllerMap::controllers["SA_E"]->refresh_quad_angle();

    publish_sa_pos_data();
}

void ROSHandler::InternalHandler::zero_carousel_cmd(mrover::Signal& msg) {
    ControllerMap::controllers["CAROUSEL_MOTOR"]->zero();
}
