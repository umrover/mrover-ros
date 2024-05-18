#include "object_detector.hpp"
#include "lie.hpp"
#include <bitset>
#include <sensor_msgs/Image.h>

namespace mrover {
    auto ObjectDetectorNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if constexpr (mEnableLoopProfilerPC) {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
            mLoopProfilerPC.beginLoop();
            mLoopProfilerPC.measureEvent("Wait");
        }

        // Adjust the picture size to be in line with the expected img size from the Point Cloud
        if (static_cast<int>(msg->height) != mImgPC.rows || static_cast<int>(msg->width) != mImgPC.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImgPC.cols, mImgPC.rows, msg->width, msg->height);
            mImgPC = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, cv::Scalar{0, 0, 0, 0}};
        }

        // Convert the pointcloud data into rgba image and store in mImg
        convertPointCloudToRGBA(msg, mImgPC);

        // Resize the image and change it from BGRA to BGR
        cv::Mat sizedImage;
        cv::Size imgSize{640, 640};
        cv::resize(mImgPC, sizedImage, imgSize);
        cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGRA2BGR);

        // Create the blob from the resized image
        cv::dnn::blobFromImage(sizedImage, mImageBlobPC, 1.0 / 255.0, imgSize, cv::Scalar{}, true, false);

        if constexpr (mEnableLoopProfilerPC) {
            mLoopProfilerPC.measureEvent("Convert Image");
        }

        // Run the blob through the model    
        std::vector<Detection> detections{};
        mLearningPC.modelForwardPass(mImageBlobPC, detections);     
        
        if constexpr (mEnableLoopProfilerPC) {
            mLoopProfilerPC.measureEvent("Execute on GPU");
        }
      
        // Increment Object hit counts if theyre seen
        // Decrement Object hit counts if they're not seen
        updateHitsObject(msg, detections); 

        // Draw the bounding boxes on the image
        drawOnImage(sizedImage, detections);        

        if constexpr (mEnableLoopProfilerPC) {
            mLoopProfilerPC.measureEvent("Push to TF");
        }

        // We only want to publish the debug image if there is something lsitening, to reduce the operations going on
        if (mDebugImgPubPC.getNumSubscribers()) {
            // Publishes the image to the debug publisher
            publishImg(sizedImage);
        }

        if constexpr (mEnableLoopProfilerPC) {
            mLoopProfilerPC.measureEvent("Publish Debug Img");
        }
    }

    auto ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if constexpr (mEnableLoopProfilerIMG) {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
            mLoopProfilerIMG.beginLoop();
            mLoopProfilerIMG.measureEvent("Wait");
        }

        // Adjust the picture size to be in line with the expected img size from the Point Cloud
        if (static_cast<int>(msg->height) != mImgIMG.rows || static_cast<int>(msg->width) != mImgIMG.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImgIMG.cols, mImgIMG.rows, msg->width, msg->height);
            mImgIMG = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())};
        }

        // Resize the image and change it from BGRA to BGR
        cv::Mat sizedImage;
        cv::Size imgSize{640, 640};
        cv::resize(mImgIMG, sizedImage, imgSize);
        cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGRA2BGR);

        // Create the blob from the resized image
        cv::dnn::blobFromImage(sizedImage, mImageBlobIMG, 1.0 / 255.0, imgSize, cv::Scalar{}, true, false);

        if constexpr (mEnableLoopProfilerIMG) {
            mLoopProfilerIMG.measureEvent("Convert Image");
        }

        // Run the blob through the model    
        std::vector<Detection> detections{};
        mLearningIMG.modelForwardPass(mImageBlobIMG, detections);     
        
        if constexpr (mEnableLoopProfilerIMG) {
            mLoopProfilerIMG.measureEvent("Execute on GPU");
        }
      
        // Increment Object hit counts if theyre seen
        // Decrement Object hit counts if they're not seen
        // TODO(LONG RANGE CAMERA): recreate this function to accept images
        //updateHitsObject(msg, detections); 

        // Draw the bounding boxes on the image
        drawOnImage(sizedImage, detections);        

        if constexpr (mEnableLoopProfilerIMG) {
            mLoopProfilerIMG.measureEvent("Push to TF");
        }

        // We only want to publish the debug image if there is something lsitening, to reduce the operations going on
        if (mDebugImgPubIMG.getNumSubscribers()) {
            // Publishes the image to the debug publisher
            publishImg(sizedImage);
        }

        if constexpr (mEnableLoopProfilerIMG) {
            mLoopProfilerIMG.measureEvent("Publish Debug Img");
        }
    }

    auto ObjectDetectorNodelet::drawOnImage(cv::Mat& image, const std::vector<Detection>& detections) -> void{
        // Draw the detected object's bounding boxes on the image for each of the objects detected
        const std::array fontColors{cv::Scalar{232, 115, 5}, cv::Scalar{0, 4, 227}};
        for (std::size_t i = 0; i < detections.size(); i++) {
            // Font color will change for each different detection
            const cv::Scalar& fontColor = fontColors.at(detections[i].classId);
            cv::rectangle(image, detections[i].box, fontColor, 1, cv::LINE_8, 0);

            // Put the text on the image
            cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
            constexpr int fontSize = 1;
            constexpr int fontWeight = 2;
            putText(image, detections[i].className, textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, fontColor, fontWeight); // Putting the text in the matrix
        }
    }

    auto ObjectDetectorNodelet::getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v, size_t width, size_t height) -> std::optional<SE3d> {
        assert(cloudPtr);
        if (u >= cloudPtr->width || v >= cloudPtr->height) {
            NODELET_WARN("Tag center out of bounds: [%zu %zu]", u, v);
            return std::nullopt;
        }

        //Search for the pnt in a spiral pattern
        return spiralSearchInImg(cloudPtr, u, v, width, height);
    }

    auto ObjectDetectorNodelet::spiralSearchInImg(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t xCenter, size_t yCenter, size_t width, size_t height) -> std::optional<SE3d> {
        std::size_t currX = xCenter;
        std::size_t currY = yCenter;
        std::size_t radius = 0;
        int t = 0;
        constexpr int numPts = 16;
        bool isPointInvalid = true;
        Point point{};

        // Find the smaller of the two box dimensions so we know the max spiral radius
        std::size_t smallDim = std::min(width / 2, height / 2);

        while (isPointInvalid) {
            // This is the parametric equation to spiral around the center pnt
            currX = static_cast<size_t>(static_cast<double>(xCenter) + std::cos(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));
            currY = static_cast<size_t>(static_cast<double>(yCenter) + std::sin(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));

            // Grab the point from the pntCloud and determine if its a finite pnt
            point = reinterpret_cast<Point const*>(cloudPtr->data.data())[currX + currY * cloudPtr->width];
            isPointInvalid = !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
            if (isPointInvalid)
                NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);

            // After a full circle increase the radius
            if (t % numPts == 0) {
                radius++;
            }

            // Increase the parameter
            t++;

            // If we reach the edge of the box we stop spiraling
            if (radius >= smallDim) {
                return std::nullopt;
            }
        }

        return std::make_optional<SE3d>(R3{point.x, point.y, point.z}, SO3d::Identity());
    }

    auto ObjectDetectorNodelet::convertPointCloudToRGBA(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat& img) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec4b*>(img.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + img.total(), [&](cv::Vec4b& pixel) {
            size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
            pixel[3] = pointPtr[i].a;
        });
    }

    auto ObjectDetectorNodelet::convertImageToRGBA(sensor_msgs::ImageConstPtr const& msg, cv::Mat& img) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec4b*>(img.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + img.total(), [&](cv::Vec4b& pixel) {
            size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
            pixel[3] = pointPtr[i].a;
        });
    }

    auto ObjectDetectorNodelet::updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg, const std::vector<Detection>& detections, cv::Size const& imgSize) -> void {
        // Set of flags indicating if the given object has been seen
        std::bitset<2> seenObjects{0b00};
        for (Detection const& detection: detections) {
            cv::Rect const& box = detection.box;
            auto center = std::pair(box.x + box.width / 2, box.y + box.height / 2);
            // Resize from {640, 640} image space to {720,1280} image space
            auto centerWidth = static_cast<std::size_t>(center.first * static_cast<double>(msg->width) / imgSize.width);
            auto centerHeight = static_cast<std::size_t>(center.second * static_cast<double>(msg->height) / imgSize.height);

            assert(static_cast<std::size_t>(detection.classId) < mObjectHitCounts.size());

            if (seenObjects[detection.classId]) return;

            seenObjects[detection.classId] = true;

            // Get the object's position in 3D from the point cloud and run this statement if the optional has a value
            if (std::optional<SE3d> objectInCamera = getObjectInCamFromPixel(msg, centerWidth, centerHeight, box.width, box.height)) {
                try {
                    std::string objectImmediateFrame = std::format("immediate{}", detection.className);
                    // Push the immediate detections to the camera frame
                    SE3Conversions::pushToTfTree(mTfBroadcaster, objectImmediateFrame, mCameraFrameId, objectInCamera.value());
                    // Since the object is seen we need to increment the hit counter
                    mObjectHitCounts[detection.classId] = std::min(mObjMaxHitcount, mObjectHitCounts[detection.classId] + mObjIncrementWeight);

                    // Only publish to permament if we are confident in the object
                    if (mObjectHitCounts[detection.classId] > mObjHitThreshold) {
                        std::string objectPermanentFrame = detection.className;
                        // Grab the object inside of the camera frame and push it into the map frame
                        SE3d objectInMap = SE3Conversions::fromTfTree(mTfBuffer, objectImmediateFrame, mMapFrame);
                        SE3Conversions::pushToTfTree(mTfBroadcaster, objectPermanentFrame, mMapFrame, objectInMap);
                    }

                } catch (tf2::ExtrapolationException const&) {
                    NODELET_WARN("Old data for immediate tag");
                } catch (tf2::LookupException const&) {
                    NODELET_WARN("Expected transform for immediate tag");
                } catch (tf::ConnectivityException const&) {
                    NODELET_WARN("Expected connection to odom frame. Is visual odometry running?");
                }
            }
        }

        for (std::size_t i = 0; i < seenObjects.size(); i++) {
            if (seenObjects[i]) continue;

            assert(i < mObjectHitCounts.size());
            mObjectHitCounts[i] = std::max(0, mObjectHitCounts[i] - mObjDecrementWeight);
        }
    }

    auto ObjectDetectorNodelet::publishImg(cv::Mat const& img) -> void {
        sensor_msgs::Image newDebugImageMessage; // I chose regular msg not ptr so it can be used outside of this process

        // Convert the image back to BGRA for ROS
        cv::Mat bgraImg;
        cv::cvtColor(img, bgraImg, cv::COLOR_BGR2BGRA);

        newDebugImageMessage.height = bgraImg.rows;
        newDebugImageMessage.width = bgraImg.cols;
        newDebugImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
        newDebugImageMessage.step = bgraImg.channels() * bgraImg.cols;
        newDebugImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;

        auto imgPtr = bgraImg.data;

        //Calculate the image size
        size_t size = newDebugImageMessage.step * newDebugImageMessage.height;
        newDebugImageMessage.data.resize(size);

        //Copy the data to the image
        std::memcpy(newDebugImageMessage.data.data(), imgPtr, size);

        mDebugImgPubPC.publish(newDebugImageMessage);
    }

} // namespace mrover