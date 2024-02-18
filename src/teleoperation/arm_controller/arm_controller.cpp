#include "arm_controller.hpp"
#include <lie/lie.hpp>
#include <ros/init.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mrover {

    // Constants

    // From: arm.urdf.xacro
    constexpr double LINK_AB = 0.58;
    constexpr double LINK_BC = 0.55;
    constexpr double LINK_CD = 0.142;
    constexpr double TAU = std::numbers::pi * 2;
    double const OFFSET = std::atan2(0.09, LINK_BC);
    constexpr double JOINT_A_MIN = -1;
    constexpr double JOINT_A_MAX = 1;
    constexpr double JOINT_B_MIN = -0.5 * std::numbers::pi;
    constexpr double JOINT_B_MAX = 0;
    constexpr double JOINT_C_MIN = 0;
    constexpr double JOINT_C_MAX = std::numbers::pi;
    constexpr double JOINT_DE_PITCH_MIN = -1 * std::numbers::pi;
    constexpr double JOINT_DE_PITCH_MAX = std::numbers::pi;
    constexpr double JOINT_DE_ROLL_MIN = -1 * std::numbers::pi;
    constexpr double JOINT_DE_ROLL_MAX = std::numbers::pi;

    // Subscribers


    // Publishers



    // Private state

    // auto ArmController::run(int argc, char** argv) -> int {
    //     ros::init(argc, argv, "arm_controller");
    //     ros::NodeHandle nh;

    //     double frequency{};
    //     nh.param<double>("/frequency", frequency, 100);

    //     ik_subscriber = nh.subscribe("arm_ik", 1, &ArmController::ik_callback, this);
    //     position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

    //     ros::spin();

    //     return EXIT_SUCCESS;
    // }

    auto ArmController::ik_callback(IK const& ik_target) -> void {
        Position positions;
        positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        positions.positions.resize(positions.names.size(), 0.f);

        double x = ik_target.pose.position.x;
        double y = ik_target.pose.position.y;
        double z = ik_target.pose.position.z;
        SE3d pos{{x, y, z}, SO3d::Identity()};
        SE3Conversions::pushToTfTree(tfBroadcaster, "arm_target", "chassis_link", pos);

        // double C = x * x + z * z - LINK_AB * LINK_AB - LINK_BC * LINK_BC;
        // double q2 = std::acos(C / (2 * LINK_AB * LINK_BC));
        // double q1 = std::atan2(z, x) - std::atan2(LINK_BC * std::sin(q2), LINK_AB + LINK_BC * std::cos(q2));
        // // q1 = std::clamp(q1, -TAU / 8, 0.0);
        // double q3 = -(q1 + q2);

        // ROS_INFO("x: %f, y: %f, z: %f, q1: %f, q2: %f, q3: %f", x, y, z, q1, q2, q3);

        double gamma = 0;
        double x3 = x - LINK_CD * cos(gamma);
        double z3 = z - LINK_CD * sin(gamma);

        double C = sqrt(x3 * x3 + z3 * z3);
        double alpha = acos((LINK_AB * LINK_AB + LINK_BC * LINK_BC - C * C) / (2 * LINK_AB * LINK_BC));
        double beta = acos((LINK_AB * LINK_AB + C * C - LINK_BC * LINK_BC) / (2 * LINK_AB * C));
        double thetaA = atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB;
        double q3 = -thetaC;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3) && 
            y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
            q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
            q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX
            ) {
            positions.positions[0] = static_cast<float>(y);
            positions.positions[1] = static_cast<float>(q1);
            positions.positions[2] = static_cast<float>(q2);
            positions.positions[3] = static_cast<float>(q3);
            position_publisher.publish(positions);
        } else {
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "arm_controller");
    mrover::ArmController arm_controller{};

    ros::spin();
    // return arm_controller.run(argc, argv);
}
