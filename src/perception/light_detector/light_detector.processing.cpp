#include "light_detector.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace mrover {
    auto LightDetector::rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d{
        double r = static_cast<double>(rgb[0]) / 255;
        double g = static_cast<double>(rgb[1]) / 255;
        double b = static_cast<double>(rgb[2]) / 255;
        double maxc = std::max(r, std::max(g, b));
        double minc = std::min(r, std::min(g, b));
        double v = maxc;

        if(minc == maxc)
            return {0.0, 0.0, v};

        double s = (maxc-minc) / maxc;
        double rc = (maxc-r) / (maxc-minc);
        double gc = (maxc-g) / (maxc-minc);
        double bc = (maxc-b) / (maxc-minc);
        double h = 0;

        if(r == maxc){
            h = 0.0+bc-gc;
        }else if(g == maxc){
            h = 2.0+rc-bc;
        }else{
            h = 4.0+gc-rc;
        }

        h = (h/6.0) - static_cast<int>(h/6.0); // get decimal

        return {h * 360, s * 100, v * 100};
    } 
	auto LightDetector::imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void{
		if(mImgRGB.rows != static_cast<int>(msg->height) || mImgRGB.cols != static_cast<int>(msg->width)){
			ROS_INFO_STREAM("Adjusting Image Size... " << msg->width << ", " << msg->height);
			mImgRGB = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, cv::Scalar::zeros()};
			mImgHSV = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, cv::Scalar::zeros()};
			mThresholdedImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC1, cv::Scalar::zeros()};
		}

		convertPointCloudToRGB(msg, mImgRGB);

        cv::Vec3d lowerBound1{0, 50, 50};
        cv::Vec3d upperBound1{20, 100, 100};

        cv::Vec3d lowerBound2{340, 50, 50};
        cv::Vec3d upperBound2{360, 100, 100};

        auto* first = reinterpret_cast<cv::Vec3b*>(mImgRGB.data);
        auto* last = reinterpret_cast<cv::Vec3b*>(first + mImgRGB.total());

        auto less = [](cv::Vec3d const& lhs, cv::Vec3d const& rhs){
            return lhs[0] < rhs[0] && lhs[1] < rhs[1] && lhs[2] < rhs[2];
        };

        auto greater = [](cv::Vec3d const& lhs, cv::Vec3d const& rhs){
            return lhs[0] > rhs[0] && lhs[1] > rhs[1] && lhs[2] > rhs[2];
        };

        std::for_each(std::execution::par_unseq, first, last, [&](cv::Vec3b& pixel){
            std::size_t const i = &pixel - first;

            cv::Vec3b pixelHSV = rgb_to_hsv(pixel);

            if((less(lowerBound1, pixelHSV) && greater(upperBound1, pixelHSV)) || (less(lowerBound2, pixelHSV) && greater(upperBound2, pixelHSV))){
                mThresholdedImg.data[i] = 255;
            }else{
                mThresholdedImg.data[i] = 0;
            }
        });
        cv::Mat erode;

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::erode(erode, mThresholdedImg, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
		
        cv::cvtColor(erode, mOutputImage, cv::COLOR_GRAY2BGRA);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(erode, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		// Find the centroids for all of the different contours
		std::vector<std::pair<int, int>> centroids; // These are in image space
		centroids.resize(contours.size());
		for(std::size_t i = 0; i < contours.size(); ++i){
			auto const& vec = contours[i];
			auto& centroid = centroids[i]; // first = row, second = col

			for(auto const& point : vec){
				centroid.first += point.y;
				centroid.second += point.x;
			}

			// This is protected division since vec will only exist if it contains points
			centroid.first /= static_cast<int>(vec.size());
			centroid.second /= static_cast<int>(vec.size());

			ROS_INFO_STREAM(std::format("centroid at (row, col) ({}, {})", centroid.first, centroid.second));
		}

		ROS_INFO_STREAM(contours.size());

		publishDetectedObjects(mOutputImage);
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

		// Load the data into the message in the correct format
        cv::Mat debugImageWrapper{image.size(), CV_8UC4, imgMsg.data.data()};
        cv::cvtColor(image, debugImageWrapper, cv::COLOR_RGB2BGRA);

        imgPub.publish(imgMsg);
    }
} //mrover
