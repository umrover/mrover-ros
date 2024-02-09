#include "inference_wrapper.hpp"

#include "inference.cuh"

using namespace nvinfer1;

/**
* cudaMemcpys CPU memory in inputTensor to GPU based on bindings
* Queues that tensor to be passed through model
* cudaMemcpys the result back to CPU memory
* Requires bindings, inputTensor, stream
* Modifies stream, outputTensor
*/
namespace mrover {

    InferenceWrapper::InferenceWrapper(std::string onnxModelPath, std::string const& modelName) {
        mInference.reset(new Inference(std::move(onnxModelPath), modelName));
    }

    auto InferenceWrapper::doDetections(cv::Mat const& img) const -> void {
        // Execute the forward pass on the inference object
        mInference->doDetections(img);
    }

    auto InferenceWrapper::getOutputTensor() const -> cv::Mat {
        return mInference->getOutputTensor();
    }

} // namespace mrover