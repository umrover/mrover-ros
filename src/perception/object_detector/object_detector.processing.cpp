#include "object_detector.hpp"

#include "inference_wrapper.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include "../point.hpp"


namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        if (static_cast<int>(msg->height) != mImg.rows || static_cast<int>(msg->width) != mImg.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImg.cols, mImg.rows, msg->width, msg->height);
            mImg = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, cv::Scalar{0, 0, 0, 0}};
        }
        auto* pixelPtr = reinterpret_cast<cv::Vec4b*>(mImg.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + mImg.total(), [&](cv::Vec4b& pixel) {
            size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
            pixel[3] = pointPtr[i].a;
        });

        cv::Mat sizedImage;

        cv::resize(mImg, sizedImage, cv::Size(640, 640));
        cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGRA2BGR);

        cv::dnn::blobFromImage(sizedImage, mImageBlob, 1.0 / 255.0, cv::Size{640, 640}, cv::Scalar(), true, false);

        mInferenceWrapper.doDetections(mImageBlob);

        cv::Mat output = mInferenceWrapper.getOutputTensor();

        int rows = output.rows;
        int dimensions = output.cols;

        bool yolov8 = false;
        // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
        // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
        if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
        {
            yolov8 = true;
            rows = output.cols;
            dimensions = output.rows;

            output = output.reshape(1, dimensions);
            cv::transpose(output, output);
        }
        auto data = reinterpret_cast<float*>(output.data);

        //Model Information
        float modelInputCols = 640;
        float modelInputRows = 640;
        float modelShapeWidth = 640;
        float modelShapeHeight = 640;

        float modelScoreThreshold = 0.90;
        float modelNMSThreshold = 0.50;


        float x_factor = modelInputCols / modelShapeWidth;
        float y_factor = modelInputRows / modelShapeHeight;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (int i = 0; i < rows; ++i) {

            if (yolov8) {
                float* classes_scores = data + 4;

                cv::Mat scores(1, static_cast<int>(classes.size()), CV_32FC1, classes_scores);
                cv::Point class_id;
                double maxClassScore;

                minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &class_id);

                if (maxClassScore > modelScoreThreshold) {
                    confidences.push_back(static_cast<float>(maxClassScore));
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = static_cast<int>((x - 0.5 * w) * x_factor);
                    int top = static_cast<int>((y - 0.5 * h) * y_factor);

                    int width = static_cast<int>(w * x_factor);
                    int height = static_cast<int>(h * y_factor);

                    boxes.emplace_back(left, top, width, height);
                }
            } else // yolov5
            {

                float modelConfidenceThresholdl = 0.9;

                if (float confidence = data[4]; confidence >= modelConfidenceThresholdl) {
                    float* classes_scores = data + 5;

                    cv::Mat scores(1, static_cast<int>(classes.size()), CV_32FC1, classes_scores);
                    cv::Point class_id;
                    double max_class_score;

                    minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);

                    if (max_class_score > modelScoreThreshold) {
                        confidences.push_back(confidence);
                        class_ids.push_back(class_id.x);

                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];

                        int left = static_cast<int>((x - 0.5 * w) * x_factor);
                        int top = static_cast<int>((y - 0.5 * h) * y_factor);

                        int width = static_cast<int>(w * x_factor);
                        int height = static_cast<int>(h * y_factor);

                        boxes.emplace_back(left, top, width, height);
                    }
                }
            }

            data += dimensions;
        }

        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

        std::vector<Detection> detections{};
        for(int idx : nms_result) {
            Detection result;
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(100, 255);
            result.color = cv::Scalar(dis(gen),
                                      dis(gen),
                                      dis(gen));

            result.className = classes[result.class_id];
            result.box = boxes[idx];

            detections.push_back(result);
        }
        if (!detections.empty()) {
            Detection firstDetection = detections[0];

            float classConfidence = firstDetection.confidence;

            cv::Rect box = firstDetection.box;

            DetectedObject msgData;
            msgData.object_type = firstDetection.className;
            msgData.detection_confidence = classConfidence;

            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            std::pair center(box.x + box.width/2, box.y + box.height/2);

            ROS_INFO("center image coordinates x y: %d %d", center.first * static_cast<float>(msg->width) / sizedImage.cols, center.second * static_cast<float>(msg->height) / sizedImage.rows);

            if (std::optional<SE3> objectLocation = getObjectInCamFromPixel(msg, center.first * static_cast<float>(msg->width) / sizedImage.cols, center.second * static_cast<float>(msg->height) / sizedImage.rows); objectLocation) {
                ROS_INFO("x y z position %f %f %f", objectLocation->position().x(), objectLocation->position().y(), objectLocation->position().z());

                // Publish tag to immediate
                try{
                    std::string immediateFrameId = "detectedobject";
                    SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, objectLocation.value());

                    SE3 tagInsideCamera = SE3::fromTfTree(mTfBuffer, mMapFrameId, immediateFrameId);

                    SE3::pushToTfTree(mTfBroadcaster, "detectedobjectFR", mMapFrameId, tagInsideCamera);
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

            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            //Get the heading
            float objectHeading;
            float zedFOV = 54; //54 @ 720; 42 @ 1080
            float fovPerPixel = (float) zedFOV / static_cast<float>(modelShapeWidth);
            float xCenter = static_cast<float>(box.x) + (static_cast<float>(box.width) / 2) - (static_cast<float>(modelShapeWidth) / 2);
            objectHeading = xCenter * fovPerPixel;
            msgData.bearing = objectHeading;

            //publish the data to NAV
            mDetectionData.publish(msgData);

            for (size_t i = 0; i < detections.size(); i++) {
                std::cout << detections[i].className
                          << i << std::endl;

                cv::Rect tempRect(center.first, center.second, 10, 10);
                cv::rectangle(sizedImage, detections[i].box, cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);
                cv::rectangle(sizedImage, tempRect, cv::Scalar(0, 0, 0), 3, cv::LINE_8, 0);

                //Put the text on the image
                cv::Point text_position(80, static_cast<int>(80 * (i + 1)));
                int font_size = 1;
                cv::Scalar font_Color(0, 0, 0);
                int font_weight = 2;
                putText(sizedImage, detections[i].className, text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_Color, font_weight); //Putting the text in the matrix//
            }
            //Show the image

            //Print the type of objected detected
        }
        if (mDebugImgPub.getNumSubscribers() > 0 || true) {
            ROS_INFO("Publishing Debug Img");

            // Create sensor msg image
            sensor_msgs::Image newDebugImageMessage;

            cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGR2BGRA);

            newDebugImageMessage.height = sizedImage.rows;
            newDebugImageMessage.width = sizedImage.cols;

            newDebugImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

            //Calculate the step for the imgMsg
            newDebugImageMessage.step = sizedImage.channels() * sizedImage.cols;
            newDebugImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;

            auto imgPtr = sizedImage.data;

            size_t size = newDebugImageMessage.step * newDebugImageMessage.height;
            newDebugImageMessage.data.resize(size);

            memcpy(newDebugImageMessage.data.data(), imgPtr, size);

            mDebugImgPub.publish(newDebugImageMessage);
        }
    }
    std::optional<SE3> ObjectDetectorNodelet::getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v) const {
        assert(cloudPtr);

        if (u >= cloudPtr->width || v >= cloudPtr->height) {
            NODELET_WARN("Tag center out of bounds: [%zu %zu]", u, v);
            return std::nullopt;
        }

        Point point = reinterpret_cast<Point const*>(cloudPtr->data.data())[u + v * cloudPtr->width ];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);
            return std::nullopt;
        }

        return std::make_optional<SE3>(R3{point.x, point.y, point.z}, SO3{});
    }
} // namespace mrover
