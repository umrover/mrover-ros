#include "arm_controller.hpp"

namespace mrover {

    ros::Subscriber ikSub;

    int init(int argc, char** argv) {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;
        ikSub = nh.subscribe("ik", 1, ikCallback);
        ros::spin();
        return EXIT_SUCCESS;
    }

    void ikCallback(IK const& ik) {
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::init(argc, argv);
}