#include "parser.h"

Parser::Parser(std::vector<float>& tensor) {
    this->tensor = tensor;
}

Parser::parseTensor() {
    int cols = 8400;     //Number of outputs in model
    int dimension = 84;  //80 + 4
    int numClasses = 80; //The number of possible classes

    //Output vector of detections
    std::vector<Detection> detections;

    //Intermediary vectors of data
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int x = 0; x < cols; x++) {
        int ptr = x;

        //Create box
        float l = tensor[ptr];
        ptr += cols;
        float h = tensor[ptr];
        ptr += cols;
        float width = tensor[ptr];
        ptr += cols;
        float height = tensor[ptr];

        int left = int((l - 0.5 * width));
        int top = int((h - 0.5 * height));


        boxes.push_back(cv::Rect(left, top, static_cast<int>(width), static_cast<int>(height)));

        //Find class id and confidence
        int maxIndex = 0;
        float maxConfidence = 0;

        for (int i = 0; i < numClasses; i++) {
            ptr += cols;
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