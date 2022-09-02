#include "perception.hpp"

// C++ Standard Library Headers, std namespace
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// OpenCV Headers, cv namespace
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

// ROS Headers, ros namespace
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/init.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_starter_project"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {
    Perception::Perception() {
        ros::NodeHandle nodeHandle("starter_project"); // Set namespace (See: http://wiki.ros.org/Names)

        image_transport::ImageTransport imageTransport(nodeHandle);
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        // Extra note: We need to use a lambda to bind "this" to "imageCallback"
        imageTransport.subscribe("camera/color/image_raw", 1,
                                 [this](sensor_msgs::ImageConstPtr const& image) {
                                     imageCallback(image);
                                 });

        mTagPublisher = nodeHandle.advertise<StarterProjectTag>("tag", 1);
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
        // TODO: implement me!
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: you can access the raw image (cv::Mat) with image->image
        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        // TODO: implement me!
        return {};
    }

    void Perception::publishTag(const StarterProjectTag& tag) {
        // TODO: implement me!
    }
} // namespace mrover
