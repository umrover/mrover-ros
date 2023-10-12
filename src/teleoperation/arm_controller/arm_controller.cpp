#include "arm_controller.hpp"

namespace mrover {

    ros::Subscriber ikSubscriber;

    ros::Publisher positionPublisher;

    int init(int argc, char** argv) {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        ikSubscriber = nh.subscribe("ik", 1, ikCallback);
        positionPublisher = nh.advertise<Position>("arm_position_cmd", 1);

        ros::spin();
        return EXIT_SUCCESS;
    }

    void ikCallback(IK const& ik) {
        // Linear
        Position linearRailPosition;
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::init(argc, argv);
}