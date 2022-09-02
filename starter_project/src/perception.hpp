#pragma once

#include <opencv2/aruco.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>

#include <mrover/StarterProjectTag.h>
#include <sensor_msgs/Image.h>

namespace mrover {

    /**
     *  Starter project perception node
     *
     *  Input: Image data
     *  Output: ArUco tag pixel coordinates that is closest to the center of the camera
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
         *
         * @param image
         */
        void imageCallback(sensor_msgs::ImageConstPtr const& image);

        /**
         *
         * @param image
         * @param tags
         */
        void findTagsInImage(cv_bridge::CvImagePtr const& image, std::vector<StarterProjectTag>& tags);

        /**
         *
         * @param tag
         */
        void publishTag(StarterProjectTag const& tag);

        /**
         *
         * @param tags
         * @return
         */
        [[nodiscard]] StarterProjectTag selectTag(std::vector<StarterProjectTag> const& tags);
    };

} // namespace mrover
