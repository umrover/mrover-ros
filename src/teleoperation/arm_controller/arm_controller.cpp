#include "arm_controller.hpp"

namespace mrover {

    // Constants

    // From: arm.urdf.xacro
    constexpr double LINK_CHASSIS_ARM = 20;
    constexpr double LINK_AB = 22.8544;
    constexpr double LINK_BC = 19.5129;
    constexpr double LINK_CD = 5.59352;

    // Subscribers

    [[maybe_unused]] ros::Subscriber ik_subscriber;

    // Publishers

    ros::Publisher position_publisher;

    // Private state

    IK ik_target;
    Position positions;

    int init(int argc, char** argv) {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        double frequency{};
        nh.param<double>("/frequency", frequency, 100.0);

        ik_subscriber = nh.subscribe("arm_ik", 1, ik_callback);
        position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

        positions.names = {"base_link_joint", "a_joint", "b_joint", "c_joint", "d_joint"};
        positions.positions.resize(positions.names.size(), 0.f);

        ros::Rate rate{frequency};
        while (ros::ok()) {
            auto const [q1, q2, q3] = solve(ik_target.pose.position.x, ik_target.pose.position.z, 0);
            if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3)) {
                positions.positions[0] = static_cast<float>(ik_target.pose.position.y + LINK_CHASSIS_ARM / 2);
                positions.positions[1] = static_cast<float>(q1);
                positions.positions[2] = static_cast<float>(q2);
                positions.positions[3] = static_cast<float>(q3);
                position_publisher.publish(positions);
            }

            rate.sleep();
            ros::spinOnce();
        }

        return EXIT_SUCCESS;
    }

    std::array<double, 3> solve(double x, double z, double theta) {
        // See: https://hive.blog/hive-196387/@juecoree/forward-and-reverse-kinematics-for-3r-planar-manipulator
        double norm_sq = x * x + z * z;
        double alpha = std::acos((norm_sq - LINK_AB * LINK_AB - LINK_BC * LINK_BC) / (2 * LINK_AB * LINK_BC));
        double beta = std::asin(LINK_BC * std::sin(alpha) / std::sqrt(norm_sq));
        double q1 = std::atan2(z, x) - beta;
        double q2 = alpha;
        double q3 = theta - q1 - q2;
        return {q1, q2, q3};
    }

    void ik_callback(IK const& new_ik_target) {
        ik_target = new_ik_target;
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::init(argc, argv);
}