#pragma once
#include "pch.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <ros/publisher.h>

namespace mrover {
    class Inference;

    class InferenceWrapper {
    private:
        //The pointer to the inference class
        std::shared_ptr<Inference> inferencePtr;

    public:
        InferenceWrapper() = default;

        ~InferenceWrapper() = default;

        //The constructor for the inferencewrapper
        InferenceWrapper(std::string onnxModelPath, cv::Size modelInputShape, std::string classesTxtFile);

        void doDetections(cv::Mat& img);
    };
} // namespace mrover