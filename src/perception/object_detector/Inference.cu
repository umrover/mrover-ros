#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "ioHelper.cuh"

#include <memory>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <ostream>
#include <string>

#include "inference.cuh"

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

    //Constructor
    //  : logger{}, inputTensor{}, outputTensor{}, referenceTensor{}, stream{}
    Inference::Inference(std::string onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string classesTxtFile = "") : logger{}, inputTensor{}, outputTensor{}, stream{} {

        enginePtr = std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>>{createCudaEngine(onnxModelPath)};
        if (!enginePtr) {
            throw std::runtime_error("Failed to create CUAD engine");
        }

        this->modelInputShape = modelInputShape;

        assert(this->enginePtr->getNbBindings() == 2);
        assert(this->enginePtr->bindingIsInput(0) ^ enginePtr->bindingIsInput(1));

        this->onnxModelPath = onnxModelPath;

        prepTensors();
    }

    // Initializes enginePtr with built engine
    ICudaEngine* Inference::createCudaEngine(std::string& onnxModelPath) {
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
        nvinfer1::IHostMemory* serializedEngine = builder->buildSerializedNetwork(*network, *config);
        nvinfer1::IRuntime* runtime = createInferRuntime(logger);
        return runtime->deserializeCudaEngine(serializedEngine->data(), serializedEngine->size());
    }

    void Inference::doDetections(cv::Mat& img) {
        //Do the forward pass on the network
        std::cout << "HI" << std::endl;
        launchInference(img.data, this->outputTensor.data);
        std::cout << *(this->outputTensor.data) << std::endl;
        //return Parser(this->outputTensor).parseTensor();
    }

    void Inference::launchInference(void* input, void* output) {
        int inputId = Inference::getBindingInputIndex(this->contextPtr.get());

        //Copy data to GPU memory
        cudaMemcpyAsync(this->bindings[inputId], input, inputEntries.d[0] * inputEntries.d[1] * inputEntries.d[2] * sizeof(float), cudaMemcpyHostToDevice, this->stream);

        //Queue the async engine process
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
        }

        inputEntries = nvinfer1::Dims3(modelInputShape.width, modelInputShape.height, 3);
        inputEntries = nvinfer1::Dims3(1, 84, 8400);
        cv::resize(outputTensor, outputTensor, cv::Size(84, 840));
    }

    int Inference::getBindingInputIndex(nvinfer1::IExecutionContext* context) {
        return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != nvinfer1::TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
    }
} // namespace mrover
