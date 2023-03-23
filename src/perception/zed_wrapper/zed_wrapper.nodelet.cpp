#include "zed_wrapper.hpp"

#include <nodelet/nodelet.h>

namespace mrover {
    class ZedNodelet : public nodelet::Nodelet {
    public:
        ZedNodelet() = default;

        ~ZedNodelet() override = default;

    private:
        void onInit() override {
            dtl = boost::make_shared<ZedNode>(getNodeHandle(), getPrivateNodeHandle());
        }

        boost::shared_ptr<ZedNode> dtl;
    };
} // namespace mrover

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ZedNodelet, nodelet::Nodelet)