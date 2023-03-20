#include "tag_detector.hpp"

#include <nodelet/nodelet.h>

namespace mrover {
    class TagDetectorNodelet : public nodelet::Nodelet {
    public:
        TagDetectorNodelet() = default;

        ~TagDetectorNodelet() = default;

    private:
        virtual void onInit() {
            dtl = boost::make_shared<TagDetectorNode>(getNodeHandle(), getPrivateNodeHandle());
        }

        boost::shared_ptr<TagDetectorNode> dtl;
    };
} // namespace mrover

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::TagDetectorNodelet, nodelet::Nodelet)