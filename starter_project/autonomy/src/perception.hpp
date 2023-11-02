#pragma once

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
#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#if __has_include(<mrover/StarterProjectTag.h>)
#include <mrover/StarterProjectTag.h>
#else
struct StarterProjectTag {};
#endif

namespace mrover {

    /**
     *  Starter project perception node
     *
     *  Input:  Image data, just RGB pixels.
     *  Output: ArUco tag pixel coordinates that is closest to the center of the camera.
     *          Also an approximation for how far away the tag is.
     */
    class Perception {
    private:
        ros::NodeHandle mNodeHandle;
        image_transport::ImageTransport mImageTransport;
        image_transport::Subscriber mImageSubscriber;
        cv::Ptr<cv::aruco::DetectorParameters> mTagDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mTagDictionary;
        std::vector<std::vector<cv::Point2f>> mTagCorners;
        std::vector<int> mTagIds;
        std::vector<StarterProjectTag> mTags;
        ros::Publisher mTagPublisher;

    public:
        Perception();

        /**
         * Called when we receive a new image message from the camera.
         * Specifically this is one frame.
         *
         * @param image
         */
        void imageCallback(sensor_msgs::ImageConstPtr const& image);

        /**
         *  Given an image, detect ArUco tags, and fill a vector full of output messages.
         *
         * @param image Image
         * @param tags  Output vector of tags
         */
        void findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags);

        /**
         * Publish our processed tag
         *
         * @param tag Selected tag message
         */
        void publishTag(StarterProjectTag const& tag);

        /**
         *  Given an ArUco tag in pixel space, find a metric for how close we are.
         *
         * @param image         Access to the raw OpenCV image as a matrix
         * @param tagCorners    4-tuple of the tag pixel coordinates representing the corners
         * @return              Closeness metric from rover to the tag
         */
        [[nodiscard]] float getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners);

        /**
         *  Given an ArUco tag in pixel space, find the approximate center in pixel space
         *
         * @param tagCorners    4-tuple of tag pixel coordinates representing the corners
         * @return              2-tuple (x,y) approximate center in pixel space
         */
        [[nodiscard]] std::pair<float, float> getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners);

        /**
         *  Select the tag closest to the center of the camera
         * 
         * @param tags          Vector of tags
         * @return              Center tag
         */
        [[nodiscard]] StarterProjectTag selectTag(std::vector<StarterProjectTag> const& tags, cv::Mat const& image);
    };

} // namespace mrover
