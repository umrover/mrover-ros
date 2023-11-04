#include <NvInfer.h>
#include <NvInferRuntimeBase.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "ioHelper.cuh"

#include <memory>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <string>

#include "inference.h"

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

    void Inference::launchInference(float* input, float* output) {
        int inputId = Inference::getBindingInputIndex(this->contextPtr.get());

        //Copy data to GPU memory
        cudaMemcpyAsync(this->bindings[inputId], input, inputEntries.d[0] * inputEntries.d[1] * inputEntries.d[2] * sizeof(float), cudaMemcpyHostToDevice, this->stream);

        //
        this->contextPtr->enqueueV3(this->stream);

        //Copy data to CPU memory
        cudaMemcpyAsync(output, this->bindings[1 - inputId], outputEntries.d[0] * outputEntries.d[1] * outputEntries.d[2] * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
    }

    /**
* Takes tensor bindings and allocates memory on the GPU for input and output tensors
* Requires enginePtr, bindings, inputTensor, and outputTensor
* Modifies bindings, inputTensor, and outputTensor
*/
    void Inference::prepTensors() {
        for (int i = 0; i < this->enginePtr->getNbIOTensors(); i++) {
            const char* tensorName = this->enginePtr->getIOTensorName(i);

            Dims dims{this->enginePtr->getTensorShape(tensorName)};

            size_t size = accumulate(dims.d + 1, dims.d + dims.nbDims, Inference::BATCH_SIZE, std::multiplies<>());
            std::vector<int> sizes = {dims.d[1], dims.d[2], dims.d[3]};


            // Create CUDA buffer for Tensor.
            cudaMalloc(&(this->bindings)[i], Inference::BATCH_SIZE * size * sizeof(float));


            //Size the tensors based on tensor type
            if (this->enginePtr->getTensorIOMode(this->enginePtr->getIOTensorName(i)) == nvinfer1::TensorIOMode::kINPUT) {
                this->inputTensor.reshape(3, sizes);
            } else if (this->enginePtr->getTensorIOMode(this->enginePtr->getIOTensorName(i)) == nvinfer1::TensorIOMode::kOUTPUT) {
                this->outputTensor.resize(size);
            }
        }
    }

    int Inference::getBindingInputIndex(nvinfer1::IExecutionContext* context) {
        return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != nvinfer1::TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
    }
} // namespace mrover
