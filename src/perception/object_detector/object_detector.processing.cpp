#include "object_detector.hpp"

#include "inference_wrapper.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <random>

namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        cv::Mat imageView{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())};
        cv::Mat sizedImage;

        cv::resize(imageView, sizedImage, cv::Size(640, 640));
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
        float* data = (float*) output.data;

        //Model Information
        float modelInputCols = 640;
        float modelInputRows = 640;
        float modelShapeWidth = 640;
        float modelShapeHeight = 640;

        float modelConfidenceThresholdl = 0.9;
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

                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double maxClassScore;

                minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

                if (maxClassScore > modelScoreThreshold) {
                    confidences.push_back(maxClassScore);
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5 * w) * x_factor);
                    int top = int((y - 0.5 * h) * y_factor);

                    int width = int(w * x_factor);
                    int height = int(h * y_factor);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            } else // yolov5
            {
                float confidence = data[4];

                if (confidence >= modelConfidenceThresholdl) {
                    float* classes_scores = data + 5;

                    cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                    cv::Point class_id;
                    double max_class_score;

                    minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                    if (max_class_score > modelScoreThreshold) {
                        confidences.push_back(confidence);
                        class_ids.push_back(class_id.x);

                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];

                        int left = int((x - 0.5 * w) * x_factor);
                        int top = int((y - 0.5 * h) * y_factor);

                        int width = int(w * x_factor);
                        int height = int(h * y_factor);

                        boxes.push_back(cv::Rect(left, top, width, height));
                    }
                }
            }

            data += dimensions;
        }

        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

        std::vector<Detection> detections{};
        for (unsigned long i = 0; i < nms_result.size(); ++i) {
            int idx = nms_result[i];

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

            float classConfidence = 0.0;
            cv::Rect box = firstDetection.box;

            DetectedObject msgData;
            msgData.object_type = firstDetection.className;
            msgData.detection_confidence = classConfidence;

            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            //Get the heading
            float objectHeading;
            float zedFOV = 54; //54 @ 720; 42 @ 1080
            float fovPerPixel = (float) zedFOV / (float) (msg->width);
            float xCenter = (float) box.x + ((float) box.width) / 2 - ((float) msg->width) / 2;
            objectHeading = xCenter * fovPerPixel;
            msgData.bearing = objectHeading;

            //publish the data to NAV
            mDetectionData.publish(msgData);

            for (size_t i = 0; i < detections.size(); i++) {
                std::cout << detections[i].className
                          << i << std::endl;


                cv::rectangle(sizedImage, detections[i].box, cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);

                //Put the text on the image
                cv::Point text_position(80, 80 * (i + 1));
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

            // auto* bgrGpuPtr = sizedImage.getPtr<sl::uchar1>(sl::MEM::GPU);
            auto imgPtr = sizedImage.data;

            size_t size = newDebugImageMessage.step * newDebugImageMessage.height;
            newDebugImageMessage.data.resize(size);

            memcpy(newDebugImageMessage.data.data(), imgPtr, size);

            mDebugImgPub.publish(newDebugImageMessage);
        }
    }
} // namespace mrover
/*
cv::rectangle(sizedImg, box, cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);

            //Put the text on the image
            cv::Point text_position(80, 80);
            int font_size = 1;
            cv::Scalar font_Color(0, 0, 0);
            int font_weight = 2;
            putText(sizedImg, msgData.object_type, text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_Color, font_weight); //Putting the text in the matrix//

            //Show the image
            cv::imshow("obj detector", sizedImg);
            cv::waitKey(1);
            //Print the type of objected detected
            ROS_INFO(firstDetection.className.c_str());
    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        //Ensure a valid message was received
        assert(msg);

        //Get a CV::Mat image view
        cv::Mat imageView{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        cv::dnn::blobFromImage(imageView, this->imageBlob);

        //Run Image Detections
        //Return type of Detection struct
        std::vector<Detection> detections = this->inference.doDetections(imageView);

        //just an array of DetectedObject
        DetectedObjects detectionsMessage;

        detectionsMessage.num_detections = static_cast<int>(detections.size());

        //Convert Vector of Detections to Output Message
        for (Detection& detection: detections) {
            DetectedObject objMsg = convertToObjMsg(detection);
            detectionsMessage.push_back(objMsg);
        }

        // detectionsMessage.header.seq = mSeqNum;
        detectionsMessage.header.stamp = ros::Time::now();
        // detectionsMessage.header.frame_id = "frame"

        this->publisher;
    }
    
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        cv::Mat sizedImg{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        cv::cvtColor(sizedImg, sizedImg, cv::COLOR_BGRA2BGR);

        std::vector<Detection> detections = inference.runInference(sizedImg);


        if (!detections.empty()) {
            Detection firstDetection = detections[0];

            float classConfidence = 0.0;
            cv::Rect box = firstDetection.box;

            DetectedObject msgData;
            msgData.object_type = firstDetection.className;
            msgData.detection_confidence = classConfidence;

            msgData.xBoxPixel = static_cast<float>(box.x);
            msgData.yBoxPixel = static_cast<float>(box.y);
            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            //Get the heading

            float objectHeading;
            float zedFOV = 54; //54 @ 720; 42 @ 1080
            float fovPerPixel = (float) zedFOV / (float) (msg->width);
            float xCenter = (float) box.x + ((float) box.width) / 2 - ((float) msg->width) / 2;
            objectHeading = xCenter * fovPerPixel;
            msgData.heading = objectHeading;


            //Put the rectangle on the image
            cv::rectangle(sizedImg, box, cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);

            //Put the text on the image
            cv::Point text_position(80, 80);
            int font_size = 1;
            cv::Scalar font_Color(0, 0, 0);
            int font_weight = 2;
            putText(sizedImg, msgData.object_type, text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_Color, font_weight); //Putting the text in the matrix//


            //Print the type of objected detected
            // ROS_INFO_STREAM(firstDetection.className.c_str());

            //Publish Dectection Data
            mDetectionData.publish(msgData);

            if (mDebugImgPub.getNumSubscribers() > 0 || true) {
                // Create sensor msg image
                sensor_msgs::Image newDebugImageMessage;

                //Change the mat to from bgr to bgra
                cv::cvtColor(sizedImg, sizedImg, cv::COLOR_BGR2BGRA);


                newDebugImageMessage.height = sizedImg.rows;
                newDebugImageMessage.width = sizedImg.cols;

                newDebugImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

                //Calculate the step for the imgMsg
                newDebugImageMessage.step = sizedImg.channels() * sizedImg.cols;
                newDebugImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;

                // auto* bgrGpuPtr = sizedImage.getPtr<sl::uchar1>(sl::MEM::GPU);
                auto imgPtr = sizedImg.data;

                size_t size = msg->step * msg->height;
                newDebugImageMessage.data.resize(size);

                memcpy(newDebugImageMessage.data.data(), imgPtr, size);

                mDebugImgPub.publish(newDebugImageMessage);
            }
            // Publish to
        }
        */
//Look at yolov8 documentation for the output matrix
/*
         * TODO(percep/obj-detector):
         * 0. Google "OpenCV DNN example." View a couple of tutorials. Most will be in Python but the C++ API is similar.
         * 1. Use cv::dnn::blobFromImage to convert the image to a blob
         * 2. Use mNet.forward to run the network
         * 3. Parse the output of the network to get the bounding boxes
         * 4. Publish the bounding boxes
         */

// namespace mrover
