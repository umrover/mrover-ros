#pragma once

#include "pch.hpp"

namespace mrover {
    class Inference;

    class InferenceWrapper {
        std::shared_ptr<Inference> mInference;

    public:
        InferenceWrapper() = default;

        ~InferenceWrapper() = default;

        InferenceWrapper(std::string onnxModelPath, cv::Size const modelInputShape, std::string classesTxtFile);

        void doDetections(cv::Mat const& img) const;

        cv::Mat getOutputTensor() const;
    };

} // namespace mrover
