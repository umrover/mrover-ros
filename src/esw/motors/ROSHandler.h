#ifndef ROSHandler_H
#define ROSHandler_H

#include "Controller.h"

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <unordered_map>

#include "ros/ros.h"

#include <mrover/ArmOpenLoopCmd.h>
#include <mrover/ArmPosition.h>
#include <mrover/Calibrated.h>
#include <mrover/CarouselOpenLoopCmd.h>
#include <mrover/CarouselPosition.h>
#include <mrover/Enable.h>
#include <mrover/HandCmd.h>
#include <mrover/MastGimbalCmd.h>
#include <mrover/SAOpenLoopCmd.h>
#include <mrover/SAPosition.h>
#include <mrover/ScienceHandCmd.h>
#include <mrover/Signal.h>

#define NOW std::chrono::high_resolution_clock::now()
using namespace rover_msgs;

/*
ROSHandler.h is responsible for handling incoming and outgoing ROS messages.
Incoming ROS messages will trigger functions which call the functions on the appropriate virtual Controllers. 
Outgoing ROS messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.
*/
class ROSHandler {
private:
    inline static std::chrono::high_resolution_clock::time_point last_heartbeat_output_time = NOW;
    inline static std::chrono::high_resolution_clock::time_point last_calib_data_output_time = NOW;

    // TODO - someone verify that this works
    inline static ros::NodeHandle* n = nullptr;
    inline static ros::Subscriber* arm_ik_cmd = nullptr;
    inline static ros::Subscriber* arm_open_loop_sub = nullptr;
    inline static ros::Subscriber* carousel_closed_loop_sub = nullptr;
    inline static ros::Subscriber* carousel_open_loop_sub = nullptr;
    inline static ros::Subscriber* carousel_zero_sub = nullptr;
    inline static ros::Subscriber* foot_open_loop_sub = nullptr;
    inline static ros::Subscriber* hand_open_loop_sub = nullptr;
    inline static ros::Subscriber* mast_gimbal_sub = nullptr;
    inline static ros::Subscriber* sa_ik_sub = nullptr;
    inline static ros::Subscriber* sa_open_loop_sub = nullptr;
    inline static ros::Subscriber* scoop_limit_switch_enable_sub = nullptr;
    inline static ros::Publisher* carousel_calib_data_pub = nullptr;
    inline static ros::Publisher* carousel_pos_data_pub = nullptr;
    inline static ros::Publisher* arm_b_calib_data_pub = nullptr;
    inline static ros::Publisher* arm_position_pub = nullptr;
    inline static ros::Publisher* sa_b_calib_data_pub = nullptr;
    inline static ros::Publisher* sa_position_pub = nullptr;

    //Empty object to pass to ROS subscribe
    class InternalHandler {
    public:
        //The following functions are handlers for the corresponding ROS messages
        void arm_closed_loop_cmd(mrover::ArmPosition& msg);
        void arm_open_loop_cmd(mrover::ArmOpenLoopCmd& msg);
        void carousel_closed_loop_cmd(mrover::CarouselPosition& msg);
        void carousel_open_loop_cmd(mrover::CarouselOpenLoopCmd& msg);
        void carousel_zero_cmd(mrover::Signal& msg);
        void hand_open_loop_cmd(mrover::HandCmd& msg);
        void mast_gimbal_cmd(mrover::MastGimbalCmd& msg);
        void refresh_arm_calib_data();
        void refresh_arm_quad_angles();
        void refresh_carousel_calib_data();
        void refresh_carousel_quad_angles();
        void refresh_sa_calib_data();
        void refresh_sa_quad_angles();
        void sa_closed_loop_cmd(mrover::SAPosition& msg);
        void sa_open_loop_cmd(mrover::SAOpenLoopCmd& msg);
        void science_hand_open_loop_cmd(mrover::ScienceHandCmd& msg);
        void scoop_limit_switch_enable_cmd(mrover::Enable& msg);
        void publish_carousel_calib_data();
        void publish_carousel_pos_data();
        void publish_arm_calib_data();
        void publish_arm_pos_data();
        void publish_sa_calib_data();
        void publish_sa_pos_data();
    };

    inline static InternalHandler* internal_object = nullptr;

public:
    //Initialize the ROS bus and subscribe to relevant channels with message handlers defined below
    static void init();

    //Handles a single incoming ROS message
    static void handle_incoming();

    //Decide whether outgoing messages need to be sent, and if so, send them
    static void handle_outgoing();
};

#endif
