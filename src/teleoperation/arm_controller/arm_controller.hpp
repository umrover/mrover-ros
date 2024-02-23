#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController {

        [[maybe_unused]] ros::Subscriber mIkSubscriber;
        ros::Publisher mPositionPublisher;
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        tf2_ros::Buffer mTfBuffer{};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

    public:
        ArmController();

        void ik_callback(IK const& new_ik_target);
    };

} // namespace mrover
