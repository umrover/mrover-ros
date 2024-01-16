#include "arm_controller.hpp"
#include "mrover/Position.h"

namespace mrover {

    // Constants

    // From: arm.urdf.xacro
    constexpr double LINK_CHASSIS_ARM = 1;
    constexpr double LINK_AB = 0.58;
    constexpr double LINK_BC = 0.55;
    constexpr double LINK_CD = 0.065;

    // Subscribers

    [[maybe_unused]] ros::Subscriber ik_subscriber;

    // Publishers

    ros::Publisher position_publisher;

    // Private state

    IK ik_target;
    Position positions;

    auto run(int argc, char** argv) -> int {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        double frequency{};
        nh.param<double>("/frequency", frequency, 100.0);

        ik_subscriber = nh.subscribe("arm_ik", 1, ik_callback);
        position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

        positions.names = {"arm_a_link", "arm_b_link", "arm_c_link", "arm_d_link", "arm_e_link"};
        positions.positions.resize(positions.names.size(), 0.f);

        ros::Rate rate{frequency};
        while (ros::ok()) {
            // 1. Check out: https://hive.blog/hive-196387/@juecoree/forward-and-reverse-kinematics-for-3r-planar-manipulator
            // 2. Create a function that takes in "ik_target" and solves for the linear joint angle (at index 0) and
            //    the angular joint angles (at indices 1, 2, and 3)
            // 3. Note that linear joint angle is the y position of the end effector (very simple)
            // 4. Note that angular will be more difficult, check the posed URL in 1 for help
            // 5. Fill in values from this function you wrote into "positions" variable
            // 6. Publish these positions. If in the sim, the sim arm bridge will subscribe to this.
            //    If on the real rover the arm bridge will subscribe to this.
            //    On the real rover it is ESW's job to achieve these desired joint angles.
            double y = ik_target.pose.position.y;

            double gamma = 0;

            // position of end of second link
            double x3 = ik_target.pose.position.x - LINK_CD * std::cos(gamma);
            double z3 = ik_target.pose.position.z - LINK_CD * std::sin(gamma);

            // double alpha = astd::cos((std::pow(x3, 2) + std::pow(z3, 2) - std::pow(LINK_AB, 2) - std::pow(LINK_BC, 2)) / (2 * LINK_AB * LINK_BC));
            // double beta = astd::sin((LINK_BC * std::sin(alpha)) / sqrt(std::pow(x3, 2) + std::pow(z3, 2)));
            // double thetaA = atan(z3 / x3) + beta;
            // double thetaB = -(3.1415 - alpha);
            // double thetaC = gamma - thetaA - thetaB;

            // double thetaB = -astd::cos((std::pow(x3, 2) + std::pow(z3, 2) - std::pow(LINK_AB, 2) - std::pow(LINK_BC, 2)) / (2 * LINK_AB * LINK_BC));
            // double thetaA = atan(z3 / x3) + atan((LINK_BC * std::sin(thetaB)) / (LINK_AB + LINK_BC * std::cos(thetaB)));
            // double thetaC = gamma - thetaA - thetaB;

            double C = sqrt(std::pow(x3, 2) + std::pow(z3, 2));
            double alpha = std::acos((std::pow(LINK_AB, 2) + std::pow(LINK_BC, 2) - std::pow(C, 2)) / (2 * LINK_AB * LINK_BC));
            double beta = std::acos((std::pow(LINK_AB, 2) + std::pow(C, 2) - std::pow(LINK_BC, 2)) / (2 * LINK_AB * C));
            double thetaA = atan(z3 / x3) + beta;
            double thetaB = -(3.1415 - alpha);
            double thetaC = gamma - thetaA - thetaB;

            positions.positions[0] = static_cast<float>(y);
            positions.positions[1] = static_cast<float>(-thetaA);
            positions.positions[2] = static_cast<float>(-thetaB);
            positions.positions[3] = static_cast<float>(-thetaC);
            // positions.positions[4] = static_cast<float>(ik_target.pose.orientation.w);

            position_publisher.publish(positions);
            rate.sleep();
            ros::spinOnce();
        }

        return EXIT_SUCCESS;
    }

    void ik_callback(IK const& new_ik_target) {
        ik_target = new_ik_target;
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::run(argc, argv);
}
