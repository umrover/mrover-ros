#include "object_detector.hpp"

#include "inference_wrapper.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include "../point.hpp"
#include <stdexcept>


namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        //Does the msg exist with a height and width
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        //Adjust the picture size to be in line with the expected img size form the Point Cloud
        if (static_cast<int>(msg->height) != mImg.rows || static_cast<int>(msg->width) != mImg.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImg.cols, mImg.rows, msg->width, msg->height);
            mImg = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, cv::Scalar{0, 0, 0, 0}};
        }

        //Convert the pointcloud data into rgba image
        auto* pixelPtr = reinterpret_cast<cv::Vec4b*>(mImg.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + mImg.total(), [&](cv::Vec4b& pixel) {
            size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
            pixel[3] = pointPtr[i].a;
        });

        //Resize the image and change it from BGRA to BGR
        cv::Mat sizedImage;
        cv::resize(mImg, sizedImage, cv::Size(640, 640));
        cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGRA2BGR);

        //Create the blob from the resized image
        cv::dnn::blobFromImage(sizedImage, mImageBlob, 1.0 / 255.0, cv::Size{640, 640}, cv::Scalar(), true, false);

        //Run the blob through the model
        mInferenceWrapper.doDetections(mImageBlob);

        //Retrieve the output from the model
        cv::Mat output = mInferenceWrapper.getOutputTensor();

        //Get model specific information
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
        //Reinterpret data from the output to be in a usable form
        auto data = reinterpret_cast<float*>(output.data);

        //Model Information
        float modelInputCols = 640;
        float modelInputRows = 640;
        float modelShapeWidth = 640;
        float modelShapeHeight = 640;

        //Set model thresholds
        float modelScoreThreshold = 0.90;
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
            //This should always be true
            if (yolov8) {
                //This is because the first 4 points are box[x,y,w,h]
                float* classes_scores = data + 4;

                //Create a mat to store all of the class scores
                cv::Mat scores(1, static_cast<int>(classes.size()), CV_32FC1, classes_scores);
                cv::Point class_id;
                double maxClassScore;

                //Find the max class score for the associated row
                minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &class_id);

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
            } else { //YOLO V5
                throw std::runtime_error("Something is wrong with interpretation"); 
            }

            data += dimensions;
        }

        //Coalesce the boxes into a smaller number of distinct boxes
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

        //Storage for the detection from the model
        std::vector<Detection> detections{};
        for(int idx : nms_result) {
            //Init the detection
            Detection result;

            //Fill in the id and confidence for the class
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];

            //Fill in the class name and box information
            result.className = classes[result.class_id];
            result.box = boxes[idx];

            //Push back the detection into the for storagevector
            detections.push_back(result);
        }

        //If there are detections locate them in 3D
        if (!detections.empty()) {
            //Get the first detection to locate in 3D
            Detection firstDetection = detections[0];

            //Get the associated confidence with the object
            float classConfidence = firstDetection.confidence;

            //Get the associated box with the object
            cv::Rect box = firstDetection.box;

            //Fill out the msgData information to be published to the topic
            DetectedObject msgData;
            msgData.object_type = firstDetection.className;
            msgData.detection_confidence = classConfidence;
            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            //Calculate the center of the box
            std::pair center(box.x + box.width/2, box.y + box.height/2);
            
            //Get the object's position in 3D from the point cloud
            if (std::optional<SE3> objectLocation = getObjectInCamFromPixel(msg, center.first * static_cast<float>(msg->width) / sizedImage.cols, center.second * static_cast<float>(msg->height) / sizedImage.rows); objectLocation) {
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

            //Get the heading
            float objectHeading;
            float zedFOV = 54; //54 @ 720; 42 @ 1080
            float fovPerPixel = (float) zedFOV / static_cast<float>(modelShapeWidth);
            float xCenter = static_cast<float>(box.x) + (static_cast<float>(box.width) / 2) - (static_cast<float>(modelShapeWidth) / 2);
            objectHeading = xCenter * fovPerPixel;
            msgData.bearing = objectHeading;

            //Publish the data to the topic
            mDetectionData.publish(msgData);

            //Draw the detected object's bounding boxes on the image for each of the objects detected
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
        }
        if (mDebugImgPub.getNumSubscribers() > 0 || true) {
            ROS_INFO("Publishing Debug Img");

            // Init sensor msg image
            sensor_msgs::Image newDebugImageMessage;//I chose regular msg not ptr so it can be used outside of this process

            //Convert the image back to BGRA for ROS
            cv::cvtColor(sizedImage, sizedImage, cv::COLOR_BGR2BGRA);

            //Fill in the msg image size
            newDebugImageMessage.height = sizedImage.rows;
            newDebugImageMessage.width = sizedImage.cols;

            //Fill in the encoding type
            newDebugImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

            //Calculate the step for the imgMsg
            newDebugImageMessage.step = sizedImage.channels() * sizedImage.cols;
            newDebugImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;

            //Pointer to the the image data
            auto imgPtr = sizedImage.data;

            //Calculate the image size
            size_t size = newDebugImageMessage.step * newDebugImageMessage.height;
            newDebugImageMessage.data.resize(size);

            //Copy the data to the image
            memcpy(newDebugImageMessage.data.data(), imgPtr, size);

            //Publish the image to the topic
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
