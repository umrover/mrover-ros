#pragma once

#include "pch.hpp"
#include <ros/node_handle.h>
#include <tf2_ros/transform_listener.h>

namespace mrover {

    class ArmController {
        [[maybe_unused]] ros::Subscriber ik_subscriber;
        ros::Publisher position_publisher;
        tf2_ros::TransformBroadcaster tfBroadcaster;
        tf2_ros::Buffer buffer{};        
        tf2_ros::TransformListener mTfListener{buffer};


    public:
        ArmController() : tfBroadcaster() {
            ros::NodeHandle nh;
            double frequency{};
            nh.param<double>("/frequency", frequency, 100);
            ik_subscriber = nh.subscribe("arm_ik", 1, &ArmController::ik_callback, this);
            position_publisher = nh.advertise<Position>("arm_position_cmd", 1);
        }
        void ik_callback(IK const& new_ik_target);
        // auto run(int argc, char** argv) -> int;
    };
    // void ik_callback(IK const& new_ik_target);

} // namespace mrover
