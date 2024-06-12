#include "light_detector.hpp"
#include <opencv2/core/types.hpp>

namespace mrover {
	auto LightDetector::imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void{
		if(mImg.rows != static_cast<int>(msg->height) || mImg.cols != static_cast<int>(msg->width)){
			ROS_INFO_STREAM("Adjusting Image Size... " << msg->width << ", " << msg->height);
			mImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, cv::Scalar{0,0,0,0}};
		}

		convertPointCloudToRGB(msg, mImg);

		publishDetectedObjects(mImg);
	}

	// TODO: (john) break this out into a utility so we dont have to copy all of this code
	auto LightDetector::convertPointCloudToRGB(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat const& image) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
    }

    auto LightDetector::publishDetectedObjects(cv::InputArray image) -> void {
        if (!imgPub.getNumSubscribers()) return;

		sensor_msgs::Image imgMsg;

        imgMsg.header.stamp = ros::Time::now();
        imgMsg.height = image.rows();
        imgMsg.width = image.cols();
        imgMsg.encoding = sensor_msgs::image_encodings::BGRA8;
        imgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        imgMsg.step = 4 * imgMsg.width;
        imgMsg.data.resize(imgMsg.step * imgMsg.height);
        cv::Mat debugImageWrapper{image.size(), CV_8UC4, imgMsg.data.data()};
        cv::cvtColor(image, debugImageWrapper, cv::COLOR_RGB2BGRA);

        imgPub.publish(imgMsg);
    }
} //mrover
