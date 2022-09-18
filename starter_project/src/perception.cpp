#include "perception.hpp"

// ROS Headers, ros namespace
#include <ros/init.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : mNodeHandle{}, mImageTransport{mNodeHandle} {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mImageTransport.subscribe("camera/color/image_raw", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv_bridge::CvImagePtr const& image, std::vector<StarterProjectTag>& tags) {
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: you can access the raw image (cv::Mat) with image->image
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: remove below & implement me!
        (void) image;
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        // TODO: remove below & implement me!
        (void) tags;

        return {};
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: remove below & implement me!
        (void) tag;
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        // TODO: remove below & implement me!
        (void) image;
        (void) tagCorners;

        return {};
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        // TODO: remove below & implement me!
        (void) tagCorners;

        return {};
    }

} // namespace mrover