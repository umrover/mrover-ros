#pragma once

#include "pch.hpp"

namespace mrover {
    class Inference;

    class InferenceWrapper {
        //Ptr to the inference
        std::shared_ptr<Inference> mInference;

    public:
        //Default Constructor for the Wrapper
        InferenceWrapper() = default;

        //Deconstructor for the Wrapper
        ~InferenceWrapper() = default;

        //Inference Wrapper Constructor
        InferenceWrapper(std::string onnxModelPath);

        //Forward Pass on the model
        void doDetections(cv::Mat const& img) const;

        //Retrieve the output tensor from the previous forward pass
        cv::Mat getOutputTensor() const;
    };

} // namespace mrover
