#include "arm_controller.hpp"
#include "lie.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ros/init.h>
#include <unistd.h>

namespace mrover {
    ArmController::ArmController() {
        ros::NodeHandle nh;
        // sleep(2);
        double frequency{};
        nh.param<double>("/frequency", frequency, 100);
        mIkSubscriber = nh.subscribe("arm_ik", 1, &ArmController::ik_callback, this);
        mPositionPublisher = nh.advertise<Position>("arm_position_cmd", 1);
    }

    auto yawSo3(double r) -> SO3d {
        auto q = Eigen::Quaterniond{Eigen::AngleAxisd{r, R3::UnitY()}};
        return {q.normalized()};
    }

    auto ArmController::ik_callback(IK const& ik_target) -> void {
        Position positions;
        positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        positions.positions.resize(positions.names.size(), 0.f);
        SE3d target_frame_to_arm_b_static;
        while (ros::ok()) {
            try {
                // ROS_INFO("%d", mTfBuffer._frameExists("joint_b_static"));
                // target_frame_to_arm_a_static = SE3Conversions::fromTfTree(mTfBuffer, ik_target.target.header.frame_id, "joint_a_static");
                target_frame_to_arm_b_static = SE3Conversions::fromTfTree(mTfBuffer, ik_target.target.header.frame_id, "arm_base_link");
                // ROS_INFO("Found tf");
                break;
            } catch (...) {
                ROS_INFO("Failed to grab tf");
                continue;
            }
        }
        Eigen::Vector4d target{ik_target.target.pose.position.x, ik_target.target.pose.position.y, ik_target.target.pose.position.z, 1};
        // double x = ik_target.target.pose.position.x;
        // double y = ik_target.target.pose.position.y;
        // double z = ik_target.target.pose.position.z;
        // maybe just add offset instead of doing multiplication (we don't do any rotation?)
        Eigen::Vector4d target_in_arm_b_static = target_frame_to_arm_b_static.transform() * target;
        double x = target_in_arm_b_static.x() - END_EFFECTOR_LENGTH; // shift back by the length of the end effector
        double z = target_in_arm_b_static.z();
        double y = target_in_arm_b_static.y();
        // ROS_INFO("x: %f, y: %f, z: %f", x, y, z);
        // SE3d pos{{target_in_arm_a_static.x(), target_in_arm_a_static.y(), target_in_arm_a_static.z()}, SO3d::Identity()};
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", target_frame_to_arm_b_static);

        // SE3d link_ab = SE3Conversions::fromTfTree(buffer, "arm_a_link", "arm_b_link");
        // SE3d link_bc = SE3Conversions::fromTfTree(buffer, "arm_b_link", "arm_c_link");
        // SE3d link_cd = SE3Conversions::fromTfTree(buffer, "arm_c_link", "arm_d_link");
        // SE3d link_de = SE3Conversions::fromTfTree(buffer, "arm_d_link", "arm_e_link");

        // SE3d offset = SE3Conversions::fromTfTree(buffer, "chassis_link", "arm_e_link").inverse() * pos;
        // ROS_INFO("x: %f, y: %f, z: %f", offset.translation().x(), offset.translation().y(), offset.translation().z());
        // SE3d thing = SE3Conversions::fromTfTree(buffer, "arm_c_link", "arm_d_link");

        // double C = x * x + z * z - LINK_BC * LINK_BC - LINK_CD * LINK_CD;
        // double q2 = std::acos(C / (2 * LINK_BC * LINK_CD));
        // double q1 = std::atan2(z, x) - std::atan2(LINK_BC * std::sin(q2), LINK_BC + LINK_CD * std::cos(q2));
        // // q1 = std::clamp(q1, -TAU / 8, 0.0);
        // double q3 = -(q1 + q2);

        // ROS_INFO("x: %f, y: %f, z: %f, q1: %f, q2: %f, q3: %f", x, y, z, q1, q2, q3);

        double gamma = 0;
        double x3 = x - LINK_DE * std::cos(gamma);
        double z3 = z - LINK_DE * std::sin(gamma);
        // SE3d joint_e_pos{{x3, 0, z3}, SO3d::Identity()};

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        SE3d joint_b_pos{{0.034346, 0, 0.049024}, SO3d::Identity()};
        SE3d joint_c_pos{{LINK_BC * cos(thetaA), 0, LINK_BC * sin(thetaA)}, yawSo3(-thetaA)};
        SE3d joint_d_pos{{LINK_CD * cos(thetaB), 0, LINK_CD * sin(thetaB)}, yawSo3(-thetaB)};
        SE3d joint_e_pos{{LINK_DE * cos(thetaC), 0, LINK_DE * sin(thetaC)}, yawSo3(-thetaC)};
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_b_target", "arm_a_link", joint_b_pos);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_c_target", "arm_b_target", joint_c_pos);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_d_target", "arm_c_target", joint_d_pos);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_e_target", "arm_d_target", joint_e_pos);

        double q1 = -thetaA;
        double q2 = -thetaB + 0.1608485915;
        double q3 = -thetaC - 0.1608485915;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3) &&
            y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
            q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
            q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX) {
            positions.positions[0] = static_cast<float>(y);
            positions.positions[1] = static_cast<float>(q1);
            positions.positions[2] = static_cast<float>(q2);
            positions.positions[3] = static_cast<float>(q3);
            mPositionPublisher.publish(positions);
        } else {
            // ROS_INFO("Can't reach arm target!");
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "arm_controller");

    mrover::ArmController armController;

    ros::spin();
}
