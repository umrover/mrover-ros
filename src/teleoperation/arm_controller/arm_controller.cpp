#include "arm_controller.hpp"

namespace mrover {

    // Constants

    constexpr double LINK_A = 22.8544;
    constexpr double LINK_B = 19.5129;

    // Subscribers

    [[maybe_unused]] ros::Subscriber ik_subscriber;

    // Publishers

    ros::Publisher position_publisher;

    // Private state

    IK ikTarget;
    Position positions;

    int init(int argc, char** argv) {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        ik_subscriber = nh.subscribe("arm_ik", 1, ik_callback);
        position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

        positions.names = {"base_link_joint", "a_joint", "b_joint", "c_joint", "d_joint"};
        positions.positions.resize(positions.names.size(), 0.f);

        ros::Rate rate{100};
        while (ros::ok()) {
            auto const [q1, q2, q3] = solve({ikTarget.pose.position.x, ikTarget.pose.position.z});
            if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3)) {
                positions.positions[0] = static_cast<float>(ikTarget.pose.position.y) + 10;
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

    std::array<double, 3> solve(Eigen::Vector2d const& target) {
        double q2 = -std::acos((target.squaredNorm() - LINK_A * LINK_A - LINK_B * LINK_B) / (2 * LINK_A * LINK_B));
        double q1 = std::atan2(target.y(), target.x()) + std::atan2(LINK_B * std::sin(q2), LINK_A + LINK_B * std::cos(q2));
        double q3 = std::numbers::pi / 8 - q1 - q2;
        return {q1, -q2, q3};
    }

    void ik_callback(IK const& newIkTarget) {
        ikTarget = newIkTarget;
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::init(argc, argv);
}