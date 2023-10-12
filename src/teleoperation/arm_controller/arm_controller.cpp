#include "arm_controller.hpp"

#include <Eigen/Core>

namespace mrover {

    ros::Subscriber ik_subscriber;

    ros::Publisher position_publisher;

    constexpr double LINK_A = 22.8544;
    constexpr double LINK_B = 19.5129;

    int init(int argc, char** argv) {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        ik_subscriber = nh.subscribe("ik", 1, ik_callback);
        position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

        ros::spin();
        return EXIT_SUCCESS;
    }

    std::array<double, 2> solve(Eigen::Vector2d const& target) {
        double q2 = -std::acos((target.squaredNorm() - LINK_A * LINK_A - LINK_B * LINK_B) / (2 * LINK_A * LINK_B));
        double q1 = std::atan2(target.y(), target.x()) + std::atan2(LINK_B * std::sin(q2), LINK_A + LINK_B * std::cos(q2));
        return {q1, q2};
    }

    void ik_callback(IK const& ik) {
        // Linear
        Position positions;
        auto push = [&](std::string const& name, double value) {
            positions.names.push_back(name);
            positions.positions.push_back(static_cast<float>(value));
        };
        push("joint_a", ik.pose.position.x);

        // Angular
        auto [q1, q2] = solve({ik.pose.position.y, ik.pose.position.z});
        push("joint_b", q1);
        push("joint_c", q2);
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::init(argc, argv);
}