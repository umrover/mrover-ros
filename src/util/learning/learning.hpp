#pragma once

#include "pch.hpp"

//Data type for detection
struct Detection {
    int classId{};
    std::string className;
    float confidence{};
    cv::Rect box;
};

class Learning {
private:
    std::string mModelName;

    std::vector<std::string> classes{"bottle", "hammer"};

    InferenceWrapper mInferenceWrapper;

    auto parseModelOutput(cv::Mat& output, 
                          std::vector<Detection>& detections, 
                          float modelScoreThreshold = 0.75, 
                          float modelNMSThreshold = 0.5) -> void;

public:
    Learning(std::string& modelName);
    ~Learning();

    auto modelForwardPass(cv::Mat& blob, 
                          std::vector<Detection>& detections, 
                          float modelScoreThreshold = 0.75, 
                          float modelNMSThreshold = 0.5) -> void;

};