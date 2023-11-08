#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "ioHelper.cuh"

#include <memory>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <string>

#include "inference.cuh"
#include "inferenceWrapper.hpp"

#include <array>
#include <string_view>
#include <vector>

using namespace nvinfer1;

/**
* Example Code: @link https://github.com/NVIDIA-developer-blog/code-samples/blob/master/posts/TensorRT-introduction/simpleOnnx_1.cpp
* IExecutionContest @link https://docs.nvidia.com/deeplearning/tensorrt/api/c_api/classnvinfer1_1_1_i_execution_context.html
* ------------------------------------------------------
* For additional context see @link https://www.edge-ai-vision.com/2020/04/speeding-up-deep-learning-inference-using-tensorrt/
*/


/**
* cudaMemcpys CPU memory in inputTensor to GPU based on bindings
* Queues that tensor to be passed through model
* cudaMemcpys the result back to CPU memory
* Requires bindings, inputTensor, stream
* Modifies stream, outputTensor
*/
namespace mrover {
    //Initialize the unique_ptr to the inference class
    InferenceWrapper::InferenceWrapper(std::string onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string classesTxtFile = "") {
        inferencePtr.reset(new Inference(onnxModelPath, modelInputShape, classesTxtFile));
    }

    void InferenceWrapper::doDetections(cv::Mat& img) {
        inferencePtr->doDetections(img);
    }
} // namespace mrover
