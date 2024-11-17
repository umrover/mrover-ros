#include "arm_controller.hpp"

namespace mrover {
    const ros::Duration ArmController::TIMEOUT = ros::Duration(1);
    
    ArmController::ArmController() {
        mIkSubscriber = mNh.subscribe("ee_pos_cmd", 1, &ArmController::ik_callback, this);
        mPositionPublisher = mNh.advertise<Position>("arm_position_cmd", 1);
        mVelSub = mNh.subscribe("ee_vel_cmd", 1, &ArmController::velCallback, this);
        mJointSub = mNh.subscribe("arm_joint_data", 1, &ArmController::fkCallback, this);
        mTimer = mNh.createTimer(ros::Duration(1.0 / 30.0), std::bind(&ArmController::timerCallback, this));
        mModeServ = mNh.advertiseService("ik_mode", &ArmController::modeCallback, this);
        mLastUpdate = ros::Time::now();
    }

    auto yawSo3(double r) -> SO3d {
        auto q = Eigen::Quaterniond{Eigen::AngleAxisd{r, R3d::UnitY()}};
        return {q.normalized()};
    }

    auto ArmController::ikCalc(SE3d target) -> std::optional<Position> {
        double x = target.translation().x();
        double y = target.translation().y();
        double z = target.translation().z();

        double gamma = target.rotation().eulerAngles(2, 1, 0)[1]; // double check this!
        double x3 = x - (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(gamma);
        double z3 = z - (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(gamma);

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB + JOINT_C_OFFSET;
        double q3 = -thetaC - JOINT_C_OFFSET;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3) &&
            y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
            q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
            q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX) {
            Position positions;
            positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch"};
            positions.positions = {
                    static_cast<float>(y),
                    static_cast<float>(q1),
                    static_cast<float>(q2),
                    static_cast<float>(q3),
            };
            return positions;
        }
        return std::nullopt;

    }

    void ArmController::velCallback(geometry_msgs::Vector3 const& ik_vel) {
        mVelTarget = {ik_vel.x, ik_vel.y, ik_vel.z};
        mLastUpdate = ros::Time::now();
    }

    void ArmController::fkCallback(sensor_msgs::JointState const& joint_state) {
        double y = joint_state.position[0];
        // joint b position
        double x = LINK_BC * std::cos(-joint_state.position[1]);
        double z = LINK_BC * std::sin(-joint_state.position[1]);
        // joint c position
        x += LINK_CD * std::cos(-joint_state.position[1] - joint_state.position[2] + JOINT_C_OFFSET);
        z += LINK_CD * std::sin(-joint_state.position[1] - joint_state.position[2] + JOINT_C_OFFSET);
        // joint de position
        x += (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(-joint_state.position[1] - joint_state.position[2] - joint_state.position[3]);
        z += (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(-joint_state.position[1] - joint_state.position[2] - joint_state.position[3]);
        mArmPos = SE3d{{x, y, z}, SO3d::Identity()};

    }

    auto ArmController::ik_callback(IK const& ik_target) -> void {
        // Position positions;
        // positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        // positions.positions.resize(positions.names.size());
        SE3d targetFrameToArmBaseLink;
        try {
            targetFrameToArmBaseLink = SE3Conversions::fromTfTree(mTfBuffer, ik_target.target.header.frame_id, "arm_base_link");
        } catch (tf2::TransformException const& exception) {
            ROS_WARN_STREAM_THROTTLE(1, std::format("Failed to get transform from {} to arm_base_link: {}", ik_target.target.header.frame_id, exception.what()));
            return;
        }
        SE3d endEffectorInTarget{{ik_target.target.pose.position.x, ik_target.target.pose.position.y, ik_target.target.pose.position.z}, SO3d::Identity()};
        SE3d endEffectorInArmBaseLink = targetFrameToArmBaseLink * endEffectorInTarget;
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", endEffectorInArmBaseLink);
        mPosTarget = SE3d{endEffectorInArmBaseLink.translation(), {0, ik_target.target.pose.orientation.x, 0,}};
        mLastUpdate = ros::Time::now();
    }

    auto ArmController::timerCallback() -> void {
        if (ros::Time::now() - mLastUpdate > TIMEOUT) {
            ROS_WARN_STREAM_THROTTLE(1, "IK Timed Out");
            return;
        }
        SE3d target;
        if (mArmMode == ArmMode::POSITION_CONTROL) {
            target = mPosTarget;
        } else {
            target = SE3d{{mArmPos.translation().x() + mVelTarget.x() * 0.1,
                           mArmPos.translation().y() + mVelTarget.y() * 0.1,
                           mArmPos.translation().z() + mVelTarget.z() * 0.1}, SO3d::Identity()};
        }
        auto positions = ikCalc(target);
        if (positions) {
            mPositionPublisher.publish(positions.value());
        } else {
            ROS_WARN_STREAM_THROTTLE(1, "IK Failed");
        }
    }

    auto ArmController::modeCallback(IkMode::Request & req, IkMode::Response & resp) -> bool {
        if (req.mode == IkMode::Request::POSITION_CONTROL) {
            ROS_INFO("IK Position Control Mode");
            mArmMode = ArmMode::POSITION_CONTROL;
        } else {
            ROS_INFO("IK Velocity Control Mode");
            mArmMode = ArmMode::VELOCITY_CONTROL;
        }
        return resp.success = true;
    }
} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "arm_controller");

    mrover::ArmController armController;

    ros::spin();
}