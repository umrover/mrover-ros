#pragma once

//OPEN CV
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

//STD
#include <vector>

//INFERENCE
#include "inference.h"

class Parser {
private:
    //The tensor to be parsed
    std::vector<float> tensor;

    //MAT VALS
    int width;

    //NET PARAMS
    float modelScoreThreshold = 0.5;
    float modelNMSThreshold = 0.50;
    std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};


public:
    Parser(std::vector<float>& tensor) {
        this->tensor = tensor;
    }

    std::vector<Detection> parseTensor() {
        int rows = 8400;     //Number of outputs in model
        int dimension = 84;  //80 + 4
        int numClasses = 80; //The number of possible classes

        //Output vector of detections
        std::vector<Detection> detections;

        //Intermediary vectors of data
        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (int x = 0; x < rows; x++) {
            int ptr = x;

            //Create box
            float l = tensor[ptr];
            ptr += rows;
            float h = tensor[ptr];
            ptr += rows;
            float width = tensor[ptr];
            ptr += rows;
            float height = tensor[ptr];

            int left = int((l - 0.5 * width));
            int top = int((h - 0.5 * height));


            boxes.push_back(cv::Rect(left, top, static_cast<int>(width), static_cast<int>(height)));

            //Find class id and confidence
            int maxIndex = 0;
            float maxConfidence = 0;

            for (int i = 0; i < numClasses; i++) {
                ptr += rows;
                if (tensor[ptr] > maxConfidence) {
                    maxConfidence = tensor[ptr];
                    maxIndex = i;
                }
            }

            confidences.push_back(maxConfidence);
            class_ids.push_back(maxIndex);
        }

        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

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

        return detections;
    }
};