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
#include <cv_bridge/cv_bridge.h>
#include <mrover/StarterProjectTag.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace mrover {

    /**
     *  Starter project perception node
     *
     *  Input:  Image data.
     *  Output: ArUco tag pixel coordinates that is closest to the center of the camera.
     *          Also an approximation for how far away the tag is.
     */
    class Perception {
    private:
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
        void findTagsInImage(cv_bridge::CvImagePtr const& image, std::vector<StarterProjectTag>& tags);

        /**
         * Publish our processed tag
         *
         * @param tag Selected tag message
         */
        void publishTag(StarterProjectTag const& tag);

        /**
         *  Given an ArUco tag in pixel space, find approximately how far away we are.
         *
         * @param image         Access to the raw OpenCV image as a matrix
         * @param tagCorners    4-tuple of the tag pixel coordinates representing the corners
         * @return              Approximate distance from rover to the tag
         */
        [[nodiscard]] float getDistanceApproxFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners);

        /**
         *  Given an ArUco tag in pixel space, find the approximate center in pixel space
         *
         * @param tagCorners    4-tuple of tag pixel coordinates representing the corners
         * @return              2-tuple (x,y) approximate center in pixel space
         */
        [[nodiscard]] std::pair<float, float> getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners);

        /**
         *
         * @param tags
         * @return
         */
        [[nodiscard]] StarterProjectTag selectTag(std::vector<StarterProjectTag> const& tags);
    };

} // namespace mrover
