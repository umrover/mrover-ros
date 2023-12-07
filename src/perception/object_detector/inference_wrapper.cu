#include "inference_wrapper.hpp"

#include <NvInferRuntimeBase.h>
#include <opencv4/opencv2/core/mat.hpp>

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

    //Initialize the unique_ptr to the inference class
    InferenceWrapper::InferenceWrapper(std::string onnxModelPath, cv::Size const modelInputShape = {640, 640}, std::string classesTxtFile = "") : mInference({}) {
        mInference.reset(new Inference(std::move(onnxModelPath), modelInputShape, std::move(classesTxtFile)));
    }

    void InferenceWrapper::doDetections(const cv::Mat& img) const {
        mInference->doDetections(img);
    }

    cv::Mat InferenceWrapper::getOutputTensor() const {
        return mInference->getOutputTensor();
    }

} // namespace mrover
