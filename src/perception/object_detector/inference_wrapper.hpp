#pragma once

#include "pch.hpp"

namespace mrover {

    class Inference;

    class InferenceWrapper {
        std::shared_ptr<Inference> mInference;

    public:
        InferenceWrapper() = default;

        ~InferenceWrapper() = default;

        explicit InferenceWrapper(std::string onnxModelPath, std::string const& modelName);

        auto doDetections(cv::Mat const& img) const -> void;

        // Retrieve the output tensor from the previous forward pass
        [[nodiscard]] auto getOutputTensor() const -> cv::Mat;
    };

} // namespace mrover
