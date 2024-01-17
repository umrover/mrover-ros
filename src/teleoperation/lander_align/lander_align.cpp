#include "lander_align.hpp"

namespace mrover {

    void LanderAlignNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "lander_align");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LanderAlignNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LanderAlignNodelet, nodelet::Nodelet)
#endif
