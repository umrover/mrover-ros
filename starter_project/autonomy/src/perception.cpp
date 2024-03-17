#include "perception.hpp"

// ROS Headers, ros namespace
#include <ros/init.h>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mNodeHandle.subscribe("camera/left/image", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        // mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    auto Perception::imageCallback(sensor_msgs::ImageConstPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat image{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                      CV_8UC3, const_cast<uint8_t*>(imageMessage->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(image, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(image, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) -> StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        return {};
    }

    auto Perception::publishTag(StarterProjectTag const& tag) -> void {
        // TODO: implement me!
    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        return {};
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        return {};
    }

} // namespace mrover
