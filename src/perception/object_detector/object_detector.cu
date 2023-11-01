#include <NvInfer.h>
#include <NvInferRuntimeBase.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "ioHelper.h"

#include <memory>
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
void Inference::launchInference() {
    int inputId = Inference::getBindingInputIndex(this->context);

    //Copy data to GPU memory
    cudaMemcpyAsync(this->bindings[inputId], this->inputTensor.data(), this->inputTensor.size() * sizeof(float), cudaMemcpyHostToDevice, this->stream);

    //
    this->context->enqueueV3(this->stream);

    //Copy data to CPU memory
    cudaMemcpyAsync(this->outputTensor.data(), this->bindings[1 - inputId], this->outputTensor.size() * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
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
        // Create CUDA buffer for Tensor.
        cudaMalloc(&(this->bindings)[i], Inference::BATCH_SIZE * size * sizeof(float));

        //Size the tensors based on tensor type
        if (this->enginePtr->getTensorIOMode(this->enginePtr->getIOTensorName(i)) == nvinfer1::TensorIOMode::kINPUT) {
            this->inputTensor.resize(size);
        } else if (this->enginePtr->getTensorIOMode(this->enginePtr->getIOTensorName(i)) == nvinfer1::TensorIOMode::kOUTPUT) {
            this->outputTensor.resize(size);
        }
    }
}

// Initializes enginePtr with built engine
void Inference::createCudaEngine(std::string& onnxModelPath) {
    // See link sfor additional context
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<nvinfer1::IBuilder, Destroy<nvinfer1::IBuilder>> builder{nvinfer1::createInferBuilder(logger)};
    std::unique_ptr<nvinfer1::INetworkDefinition, Destroy<nvinfer1::INetworkDefinition>> network{builder->createNetworkV2(explicitBatch)};
    std::unique_ptr<nvonnxparser::IParser, Destroy<nvonnxparser::IParser>> parser{nvonnxparser::createParser(*network, logger)};
    std::unique_ptr<nvinfer1::IBuilderConfig, Destroy<nvinfer1::IBuilderConfig>> config{builder->createBuilderConfig()};

    //Parse the onnx from file
    if (!parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(ILogger::Severity::kINFO))) {
        std::cout << "ERROR: could not parse input engine." << std::endl;
    }

    config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1 << 30);

    auto profile = builder->createOptimizationProfile();
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMIN, Dims4{1, 3, 256, 256});
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kOPT, Dims4{1, 3, 256, 256});
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMAX, Dims4{32, 3, 256, 256});
    config->addOptimizationProfile(profile);

    this->enginePtr = builder->buildSerializedNetwork(*network, *config);
}

int Inference::getBindingInputIndex(nvinfer1::IExecutionContext* context) {
    return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != nvinfer1::TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
}


void Inference::setUpContext(const std::string& inputFile) {
    // Read input tensor from input picture file.
    if (readTensor(inputFile, this->inputTensor) != this->inputTensor.size()) {
        std::cout << "Couldn't read input Tensor" << std::endl;
    }

    // Create Execution Context.
    this->context.reset(this->enginePtr->createExecutionContext());

    Dims dims_i{this->enginePtr->getBindingDimensions(0)};
    Dims4 inputDims{Inference::BATCH_SIZE, dims_i.d[1], dims_i.d[2], dims_i.d[3]};
    context->setBindingDimensions(0, inputDims);
}