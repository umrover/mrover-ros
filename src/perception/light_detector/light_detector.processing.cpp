#include "light_detector.hpp"
#include <manif/impl/se3/SE3.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mrover {
    auto LightDetector::rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d{
        // https://math.stackexchange.com/questions/556341/rgb-to-hsv-color-conversion-algorithm
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
		    mImgRGB = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, {0, 0, 0}};
			mImgHSV = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, {0, 0, 0}};
			mThresholdedImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC1, cv::Scalar{0}};
		}

		convertPointCloudToRGB(msg, mImgRGB);        

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
            if(i == 479*640){
                mThresholdedImg.data[i] = 0;
            }
            cv::Vec3b pixelHSV = rgb_to_hsv(pixel);

            if((less(mLowerBound, pixelHSV) && greater(mUpperBound, pixelHSV))){
                mThresholdedImg.data[i] = 255;
            }else{
                mThresholdedImg.data[i] = 0;
            }
        });
        cv::Mat erode;

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::erode(erode, mThresholdedImg, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
		
        cv::cvtColor(erode, mOutputImage, cv::COLOR_GRAY2BGRA);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(erode, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		// Find the centroids for all of the different contours
		std::vector<std::pair<int, int>> centroids; // These are in image space
		centroids.resize(contours.size());

        // The number of lights that we push into the TF
        unsigned int numLightsSeen = 0;

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

            // If the position of the light is defined, then push it into the TF tree
            std::optional<SE3d> lightInCamera = spiralSearchForValidPoint(msg, centroid.second, centroid.first, SPIRAL_SEARCH_DIM, SPIRAL_SEARCH_DIM);
            if(lightInCamera){
                ++numLightsSeen;
                std::string immediateLightFrame = std::format("immediateLight{}", numLightsSeen);
                if(lightInCamera.value().translation().norm() < mImmediateLightRange && getHitCount(lightInCamera) > mPublishThreshold){
                    std::string lightFrame = std::format("light{}", numLightsSeen);
                    SE3Conversions::pushToTfTree(mTfBroadcaster, lightFrame, mCameraFrame, lightInCamera.value());
                }
                increaseHitCount(lightInCamera);
                SE3Conversions::pushToTfTree(mTfBroadcaster, immediateLightFrame, mCameraFrame, lightInCamera.value());
            }
		}

        decreaseHitCounts();

		publishDetectedObjects(mOutputImage, centroids);
	}

    auto LightDetector::getHitCount(std::optional<SE3d> const& light) -> int{
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);

            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            int x = static_cast<int>(location.x());
            int y = static_cast<int>(location.x());

            std::pair<int, int> key{x, y};

            return mHitCounts[key];
        }
        return -1;
    }

    void LightDetector::increaseHitCount(std::optional<SE3d> const& light){
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);

            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            int x = static_cast<int>(location.x());
            int y = static_cast<int>(location.x());

            std::pair<int, int> key{x, y};

            mHitCounts[key] = std::min(mHitCounts[key] + mHitIncrease, mHitMax);
        }
    }

    void LightDetector::decreaseHitCounts(){
        for(auto& [_, hitCount] : mHitCounts){
            hitCount = std::max(hitCount - mHitDecrease, 0);
        }
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

    auto LightDetector::publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void {
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

		// Draw a marker where each centroid is in the image
		constexpr int MARKER_RADIUS = 10;
		cv::Scalar const MARKER_COLOR = {255, 0, 0, 0};

		for(auto const& centroid : centroids){
			cv::circle(debugImageWrapper, {centroid.second, centroid.first}, MARKER_RADIUS, MARKER_COLOR);
		}

        imgPub.publish(imgMsg);
    }

    auto LightDetector::spiralSearchForValidPoint(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d> {
        // See: https://stackoverflow.com/a/398302
        auto xc = static_cast<int>(u), yc = static_cast<int>(v);
        auto sw = static_cast<int>(width), sh = static_cast<int>(height);
        auto ih = static_cast<int>(cloudPtr->height), iw = static_cast<int>(cloudPtr->width);
        int sx = 0, sy = 0; // Spiral coordinates starting at (0, 0)
        int dx = 0, dy = -1;
        std::size_t bigger = std::max(width, height);
        std::size_t maxIterations = bigger * bigger;
        for (std::size_t i = 0; i < maxIterations; i++) {
            if (-sw / 2 < sx && sx <= sw / 2 && -sh / 2 < sy && sy <= sh / 2) {
                int ix = xc + sx, iy = yc + sy; // Image coordinates

                if (ix < 0 || ix >= iw || iy < 0 || iy >= ih) {
                    NODELET_WARN_STREAM(std::format("Spiral query is outside the image: [{}, {}]", ix, iy));
                    continue;
                }

                Point const& point = reinterpret_cast<Point const*>(cloudPtr->data.data())[ix + iy * cloudPtr->width];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;

                return std::make_optional<SE3d>(R3d{point.x, point.y, point.z}, SO3d::Identity());
            }

            if (sx == sy || (sx < 0 && sx == -sy) || (sx > 0 && sx == 1 - sy)) {
                dy = -dy;
                std::swap(dx, dy);
            }

            sx += dx;
            sy += dy;
        }
        return std::nullopt;
    }
} //mrover
