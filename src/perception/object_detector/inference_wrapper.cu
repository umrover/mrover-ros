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

    InferenceWrapper::InferenceWrapper(std::string onnxModelPath) : mInference({}) {
        //Initialize the unique_ptr to the inference class
        mInference.reset(new Inference(std::move(onnxModelPath)));
    }

    
    void InferenceWrapper::doDetections(const cv::Mat& img) const {
        //Execute the forward pass on the inference object
        mInference->doDetections(img);
    }

    cv::Mat InferenceWrapper::getOutputTensor() const {
        //Get the output tensor from the inference object
        return mInference->getOutputTensor();
    }

} // namespace mrover
