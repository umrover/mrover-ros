#include "arm_controller.hpp"

namespace mrover {

    ArmController::ArmController() {
        mIkSubscriber = mNh.subscribe("arm_ik", 1, &ArmController::ik_callback, this);
        mPositionPublisher = mNh.advertise<Position>("arm_position_cmd", 1);
    }

    auto yawSo3(double r) -> SO3d {
        auto q = Eigen::Quaterniond{Eigen::AngleAxisd{r, R3::UnitY()}};
        return {q.normalized()};
    }

    auto ArmController::ik_callback(IK const& ik_target) -> void {
        Position positions;
        positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        positions.positions.resize(positions.names.size());
        SE3d targetFrameToArmBaseLink;
        try {
            targetFrameToArmBaseLink = SE3Conversions::fromTfTree(mTfBuffer, ik_target.target.header.frame_id, "arm_base_link");
        } catch (tf2::TransformException const& exception) {
            ROS_WARN_STREAM_THROTTLE(1, std::format("Failed to get transform from {} to arm_base_link: {}", ik_target.target.header.frame_id, exception.what()));
            return;
        }
        SE3d endEffectorInTarget{{ik_target.target.pose.position.x, ik_target.target.pose.position.y, ik_target.target.pose.position.z}, SO3d::Identity()};
        SE3d endEffectorInArmBaseLink = targetFrameToArmBaseLink * endEffectorInTarget;
        double x = endEffectorInArmBaseLink.translation().x() - END_EFFECTOR_LENGTH; // shift back by the length of the end effector
        double y = endEffectorInArmBaseLink.translation().y();
        double z = endEffectorInArmBaseLink.translation().z();
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", endEffectorInArmBaseLink);

        double gamma = 0;
        double x3 = x - LINK_DE * std::cos(gamma);
        double z3 = z - LINK_DE * std::sin(gamma);

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB + 0.1608485915;
        double q3 = -thetaC - 0.1608485915;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3)) {
            SE3d joint_b_pos{{0.034346, 0, 0.049024}, SO3d::Identity()};
            SE3d joint_c_pos{{LINK_BC * cos(thetaA), 0, LINK_BC * sin(thetaA)}, yawSo3(-thetaA)};
            SE3d joint_d_pos{{LINK_CD * cos(thetaB), 0, LINK_CD * sin(thetaB)}, yawSo3(-thetaB)};
            SE3d joint_e_pos{{LINK_DE * cos(thetaC), 0, LINK_DE * sin(thetaC)}, yawSo3(-thetaC)};
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_b_target", "arm_a_link", joint_b_pos);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_c_target", "arm_b_target", joint_c_pos);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_d_target", "arm_c_target", joint_d_pos);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_e_target", "arm_d_target", joint_e_pos);

            if (y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
                q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
                q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
                q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX) {
                positions.positions[0] = static_cast<float>(y);
                positions.positions[1] = static_cast<float>(q1);
                positions.positions[2] = static_cast<float>(q2);
                positions.positions[3] = static_cast<float>(q3);
                mPositionPublisher.publish(positions);
            } else {
                ROS_WARN_THROTTLE(1, "Can not reach target within arm limits!");
            }
        } else {
            ROS_WARN_THROTTLE(1, "Can not solve for arm target!");
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "arm_controller");

    mrover::ArmController armController;

    ros::spin();
}