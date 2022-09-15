#include "perception.hpp"

// ROS Headers, ros namespace
#include <image_transport/image_transport.h>
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

    Perception::Perception() : mNodeHandle{} {
        image_transport::ImageTransport imageTransport(mNodeHandle);
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        imageTransport.subscribe("camera/color/image_raw", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        // mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);
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
        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds);
        for (size_t i = 0; i < mTagIds.size(); ++i) {
            auto corner = mTagCorners[i];
            auto id = mTagIds[i];
            auto[x, y] = getCenterFromTagCorners(corner);
            float dist = getClosenessMetricFromTagCorners(image->image, corner);
            StarterProjectTag tag;
            tag.id = id;
            tag.x = x;
            tag.y = y;
            tag.dist = dist;
            tags.push_back(tag);
        }

    }

    float distCenter(StarterProjectTag t){
        return sqrt(pow(t.x, 2) + pow(t.y, 2));
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        
        auto distanceCompF = [](StarterProjectTag t1, StarterProjectTag t2){
            return distCenter(t1) < distCenter(t2);
        };
        return *(std::min_element(tags.begin(), tags.end(), distanceCompF));
    }

    void Perception::publishTag(const StarterProjectTag& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        auto compXF = [](cv::Point2f p1 , cv::Point2f p2){
            return p1.x < p2.x;
        };
        auto compYF = [](cv::Point2f p1, cv::Point2f p2){
            return p1.y < p2.y;
        };
        float w = (std::max_element(tagCorners.begin(), tagCorners.end(), compXF))->x - 
                  (std::min_element(tagCorners.begin(), tagCorners.end(), compXF))->x;
        float h = (std::max_element(tagCorners.begin(), tagCorners.end(), compYF))->y - 
                  (std::min_element(tagCorners.begin(), tagCorners.end(), compYF))->y;
        return (w * h) / (image.rows * image.cols);
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(const std::vector<cv::Point2f>& tagCorners) {
        // TODO: implement me!
        float xAv, yAv;
        for (auto corner : tagCorners){
            xAv += corner.x / 4.0;
            yAv += corner.y / 4.0;
        }
        return {xAv, yAv};
    }

} // namespace mrover
