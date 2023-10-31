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
        mImageSubscriber = mImageTransport.subscribe("camera/right/image", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::aruco::DetectorParameters::create();
        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat cvImage{static_cast<int>(image->height), static_cast<int>(image->width),
                        CV_8UC3, const_cast<uint8_t*>(image->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);

        for (size_t i = 0; i < mTagIds.size(); ++i) {
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);

            tag.xTagCenterPixel = (center.first - image.cols/2) / image.cols;
            tag.yTagCenterPixel = (center.second - image.rows/2) / image.rows;
            tag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            mTags.push_back(tag);
        }
        
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        if (tags.empty()) {
            StarterProjectTag tag;
            tag.tagId = -1;
            return tag;
        } else if (tags.size() == 1) { 
            return tags[0];
        } else {
            int minTagIndex = 0;
            double minTagDist = sqrt(pow(tags[0].xTagCenterPixel, 2) + pow(tags[0].yTagCenterPixel,2));

            for (size_t i = 1; i < tags.size(); ++i) {
                double currentTagDist = sqrt(pow(tags[i].xTagCenterPixel, 2) + pow(tags[i].yTagCenterPixel,2));

                if (currentTagDist < minTagDist) {
                    minTagDist = currentTagDist;
                    minTagIndex = i;
                }
            }
            
            return tags[minTagIndex];
        } 
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        float areaImage = image.rows * image.cols;
        cv::Point2f topLeft = tagCorners[0];
        cv::Point2f topRight = tagCorners[1];
        cv::Point2f bottomLeft = tagCorners[2];

        float tagWidth = topRight.x - topLeft.x;
        float tagHeight = bottomLeft.y - topLeft.y;
        ROS_INFO("tag width: %f tag height: %f", tagWidth, tagHeight);

        float ratio = 1 - (abs(tagWidth * tagHeight) / areaImage);

        return ratio;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        int xSum = 0;
        int ySum = 0;

        for (cv::Point2f corner : tagCorners) {
            xSum += corner.x;
            ySum += corner.y;
        }

        std::pair<float, float> center;
        center.first = xSum;
        center.second = ySum;
        return center;
    }

} // namespace mrover
