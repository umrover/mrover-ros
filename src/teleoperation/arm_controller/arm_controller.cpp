#include "arm_controller.hpp"


namespace mrover {

    ArmController::ArmController() : mNodeHandle{} {
        mIkSubscriber = mNodeHandle.subscribe("ik", 1, &ArmController::ikCallback, this);
    }

    void ArmController::ikCallback(IK const& ik) {
        ROS_INFO("Received IK message");
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
}