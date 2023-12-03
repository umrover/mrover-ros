#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>

static const std::string OPENCV_WINDOW = "Image WIndow";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;

public:
    ImageConverter() : it_(nh_) {
        img_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCB, this);
        img_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCB(const sensor_msgs::ImageConstPtr& msg) {

        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }

        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception %s", e.what());
            return;
        }

        static int image_count = 0;
        std::stringstream sstream;
        sstream << "image" << image_count << ".png";

        ROS_ASSERT(cv::imwrite(sstream.str(), cv_ptr->image));
        image_count++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}