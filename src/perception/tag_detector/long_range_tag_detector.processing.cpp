#include "long_range_tag_detector.hpp"

#include "../point.hpp"

namespace mrover {

    /**
     * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
     *
     * @param msg   image message
     */
    void TagDetectorNodelet::imageCallback(sensor_msgs::Image const& msg) {
        // 1. Detect tags
        // 2. Update the hit counts of the tags in the mTags map
        // 3. We only want to publish the tags if the topic has subscribers
        if (mPublishImages && mImgPub.getNumSubscribers()) {
            // Draw the tags on the image using OpenCV
        }
    }

} // namespace mrover