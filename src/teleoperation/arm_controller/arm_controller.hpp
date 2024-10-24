#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController {

        [[maybe_unused]] ros::Subscriber mIkSubscriber;
        ros::NodeHandle mNh;
        ros::Publisher mPositionPublisher;
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        tf2_ros::Buffer mTfBuffer{};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

    public:
        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.5344417294;
        static constexpr double LINK_CD = 0.5531735368;
        static constexpr double LINK_DE = 0.044886000454425812;
        static constexpr double JOINT_A_MIN = 0;
        static constexpr double JOINT_A_MAX = 0.45;
        static constexpr double JOINT_B_MIN = -0.25 * std::numbers::pi;
        static constexpr double JOINT_B_MAX = 0;
        static constexpr double JOINT_C_MIN = 1.35;
        static constexpr double JOINT_C_MAX = 3.19;
        static constexpr double JOINT_DE_PITCH_MIN = -0.70;
        static constexpr double JOINT_DE_PITCH_MAX = 0.95;
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // measured from blender

        ArmController();

        void ik_callback(IK const& new_ik_target);
    };

} // namespace mrover