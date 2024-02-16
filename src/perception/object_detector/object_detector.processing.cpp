#include "object_detector.hpp"

namespace mrover {

    auto ObjectDetectorNodelet::imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {

        if (mEnableLoopProfiler) {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
            mLoopProfiler.beginLoop();
            mLoopProfiler.measureEvent("Wait");
        }

        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        // Adjust the picture size to be in line with the expected img size from the Point Cloud
        if (static_cast<int>(msg->height) != mImg.rows || static_cast<int>(msg->width) != mImg.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImg.cols, mImg.rows, msg->width, msg->height);
            mImg = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, cv::Scalar{0, 0, 0, 0}};
        }

        // Convert the pointcloud data into rgba image and store in mImg
        convertPointCloudToRGBA(msg, mImg);

        // Resize the image and change it from BGRA to BGR
        cv::Mat sizedImage;
        cv::Size imgSize{640, 640};
        cv::resize(mImg, sizedImage, imgSize);
        cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGRA2BGR);

        // Create the blob from the resized image
        cv::dnn::blobFromImage(sizedImage, mImageBlob, 1.0 / 255.0, imgSize, cv::Scalar(), true, false);

        if (mEnableLoopProfiler) {
            mLoopProfiler.measureEvent("Convert Image");
        }

        // Run the blob through the model
        mInferenceWrapper.doDetections(mImageBlob);

        // Retrieve the output from the model
        cv::Mat output = mInferenceWrapper.getOutputTensor();

        if (mEnableLoopProfiler) {
            mLoopProfiler.measureEvent("Execute on GPU");
        }

        // Get model specific information
        int rows = output.rows;
        int dimensions = output.cols;

        // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
        // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
        if (dimensions <= rows) // Check if the shape[2] is more than shape[1] (yolov8)
        {
            throw std::runtime_error("Something is wrong Model with interpretation");
        }

        rows = output.cols;
        dimensions = output.rows;

        output = output.reshape(1, dimensions);
        cv::transpose(output, output);
        //Reinterpret data from the output to be in a usable form
        auto data = reinterpret_cast<float*>(output.data);

        //Model Information
        auto modelInputCols = static_cast<float>(imgSize.width);
        auto modelInputRows = static_cast<float>(imgSize.height);
        auto modelShapeWidth = static_cast<float>(imgSize.width);
        auto modelShapeHeight = static_cast<float>(imgSize.height);

        //Set model thresholds
        float modelScoreThreshold = 0.50;
        float modelNMSThreshold = 0.50;

        //Get x and y scale factors
        float x_factor = modelInputCols / modelShapeWidth;
        float y_factor = modelInputRows / modelShapeHeight;

        //Init storage containers
        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        //Each of the possibilities do interpret the data
        for (int i = 0; i < rows; ++i) {
            //This is because the first 4 points are box[x,y,w,h]
            float* classes_scores = data + 4;

            //Create a mat to store all of the class scores
            cv::Mat scores(1, static_cast<int>(classes.size()), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            //Find the max class score for the associated row
            cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &class_id);

            //Determine if the class is an acceptable confidence level else disregard
            if (maxClassScore > modelScoreThreshold) {
                //Push back data points into storage containers
                confidences.push_back(static_cast<float>(maxClassScore));
                class_ids.push_back(class_id.x);

                //Get the bounding box data
                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                //Cast the corners into integers to be used on pixels
                int left = static_cast<int>((x - 0.5 * w) * x_factor);
                int top = static_cast<int>((y - 0.5 * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);

                //Push abck the box into storage
                boxes.emplace_back(left, top, width, height);
            }

            data += dimensions;
        }

        //Coalesce the boxes into a smaller number of distinct boxes
        std::vector<int> nmsResult;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nmsResult);

        //Storage for the detection from the model
        std::vector<Detection> detections{};
        for (int idx: nmsResult) {
            //Init the detection
            Detection result;

            //Fill in the id and confidence for the class
            result.classId = class_ids[idx];
            result.confidence = confidences[idx];

            //Fill in the class name and box information
            result.className = classes[result.classId];
            result.box = boxes[idx];

            //Push back the detection into the for storagevector
            detections.push_back(result);
        }

        if (mEnableLoopProfiler) {
            mLoopProfiler.measureEvent("Extract Detections");
        }

        std::vector seenObjects{false, false};
        //If there are detections locate them in 3D
        for (Detection const& detection: detections) {

            //Increment Object hit counts if theyre seen
            updateHitsObject(msg, detection, seenObjects);

            //Decrement Object hit counts if they're not seen
            for (size_t i = 0; i < seenObjects.size(); i++) {
                if (!seenObjects.at(i)) {
                    mObjectHitCounts.at(i) = std::max(0, mObjectHitCounts.at(i) - mObjDecrementWeight);
                }
            }

            //Draw the detected object's bounding boxes on the image for each of the objects detected
            std::vector fontColors{cv::Scalar{232, 115, 5},
                                   cv::Scalar{0, 4, 227}};
            for (std::size_t i = 0; i < detections.size(); i++) {
                //Font color will change for each different detection
                cv::Scalar fontColor = fontColors.at(detections.at(i).classId);
                cv::rectangle(sizedImage, detections[i].box, fontColor, 1, cv::LINE_8, 0);

                //Put the text on the image
                cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
                constexpr int fontSize = 1;
                constexpr int fontWeight = 2;
                putText(sizedImage, detections[i].className, textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, fontColor, fontWeight); //Putting the text in the matrix//
            }
        }

        if (mEnableLoopProfiler) {
            mLoopProfiler.measureEvent("Push to TF");
        }

        //We only want to publish the debug image if there is something lsitening, to reduce the operations going on
        if (mDebugImgPub.getNumSubscribers() > 0 || true) {
            //Publishes the image to the debug publisher
            publishImg(sizedImage);
        }

        if (mEnableLoopProfiler) {
            mLoopProfiler.measureEvent("Publish Debug Img");
        }
    } // namespace mrover

    auto ObjectDetectorNodelet::getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v, size_t width, size_t height) -> std::optional<SE3> {
        assert(cloudPtr);
        if (u >= cloudPtr->width || v >= cloudPtr->height) {
            NODELET_WARN("Tag center out of bounds: [%zu %zu]", u, v);
            return std::nullopt;
        }

        //Search for the pnt in a spiral pattern
        return spiralSearchInImg(cloudPtr, u, v, width, height);
    }

    auto ObjectDetectorNodelet::spiralSearchInImg(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t xCenter, size_t yCenter, size_t width, size_t height) -> std::optional<SE3> {
        size_t currX = xCenter;
        size_t currY = yCenter;
        size_t radius = 0;
        int t = 0;
        constexpr int numPts = 16;
        bool isPointInvalid = true;
        Point point{};

        // Find the smaller of the two box dimensions so we know the max spiral radius
        size_t smallDim = std::min(width / 2, height / 2);

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

        return std::make_optional<SE3>(R3{point.x, point.y, point.z}, SO3{});
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

    auto ObjectDetectorNodelet::updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg, Detection const& detection, std::vector<bool>& seenObjects, cv::Size const& imgSize) -> void {

        cv::Rect box = detection.box;
        auto center = std::pair(box.x + box.width / 2, box.y + box.height / 2);
        // Resize from {640, 640} image space to {720,1280} image space
        auto centerWidth = static_cast<size_t>(center.first * static_cast<double>(msg->width) / imgSize.width);
        auto centerHeight = static_cast<size_t>(center.second * static_cast<double>(msg->height) / imgSize.height);

        if (!seenObjects.at(detection.classId)) {
            seenObjects.at(detection.classId) = true;

            //Get the object's position in 3D from the point cloud and run this statement if the optional has a value
            if (std::optional<SE3> objectLocation = getObjectInCamFromPixel(msg, centerWidth, centerHeight, box.width, box.height); objectLocation) {
                try {
                    std::string immediateFrameId = "immediateDetectedObject" + classes.at(detection.classId);

                    //Push the immediate detections to the zed frame
                    SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, objectLocation.value());


                    //Since the object is seen we need to increment the hit counter
                    mObjectHitCounts.at(detection.classId) = std::min(mObjMaxHitcount, mObjectHitCounts.at(detection.classId) + mObjIncrementWeight);

                    //Only publish to permament if we are confident in the object
                    if (mObjectHitCounts.at(detection.classId) > mObjHitThreshold) {

                        std::string permanentFrameId = "detectedObject" + classes.at(detection.classId);


                        //Grab the object inside of the camera frame and push it into the map frame
                        SE3 objectInsideCamera = SE3::fromTfTree(mTfBuffer, mMapFrameId, immediateFrameId);
                        SE3::pushToTfTree(mTfBroadcaster, permanentFrameId, mMapFrameId, objectInsideCamera);
                    }

                } catch (tf2::ExtrapolationException const&) {
                    NODELET_WARN("Old data for immediate tag");
                } catch (tf2::LookupException const&) {
                    NODELET_WARN("Expected transform for immediate tag");
                } catch (tf::ConnectivityException const&) {
                    NODELET_WARN("Expected connection to odom frame. Is visual odometry running?");
                } catch (tf::LookupException const&) {
                    NODELET_WARN("LOOK UP NOT FOUND");
                }
            }
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

        mDebugImgPub.publish(newDebugImageMessage);
    }

} // namespace mrover