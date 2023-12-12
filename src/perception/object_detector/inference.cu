#include "inference.cuh"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <NvOnnxParser.h>
#include <cstdio>
#include <cuda_runtime_api.h>
#include <opencv4/opencv2/core/hal/interface.h>

#include "logger.cuh"

using namespace nvinfer1;/**
* cudaMemcpys CPU memory in inputTensor to GPU based on bindings
* Queues that tensor to be passed through model
* cudaMemcpys the result back to CPU memory
* Requires bindings, inputTensor, stream
* Modifies stream, outputTensor
*/
namespace mrover {

    constexpr static const char * INPUT_BINDING_NAME = "images";
    constexpr static const char * OUTPUT_BINDING_NAME = "output0";

    //Constructor
    //  : logger{}, inputTensor{}, outputTensor{}, referenceTensor{}, stream{}
    Inference::Inference(std::string const& onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string const& classesTxtFile = "") {

        mEngine = std::unique_ptr<ICudaEngine, Destroy<ICudaEngine>>{createCudaEngine(onnxModelPath)};
        if (!mEngine) throw std::runtime_error("Failed to create CUDA engine");

        mLogger.log(ILogger::Severity::kINFO, "Created CUDA Engine");

        //Check some assumptions
        if(mEngine->getNbIOTensors() != 2) throw std::runtime_error("Invalid Binding Count");
        if(mEngine->getTensorIOMode(INPUT_BINDING_NAME) != TensorIOMode::kINPUT) throw std::runtime_error("Expected Input Binding 0 Is An Input");
        if(mEngine->getTensorIOMode(OUTPUT_BINDING_NAME) != TensorIOMode::kOUTPUT) throw std::runtime_error("Expected Input Binding Input To Be 1");

        mStream.emplace();

        mLogger.log(ILogger::Severity::kINFO, "Created CUDA stream");

        setUpContext();

        prepTensors();
    }

    // Initializes enginePtr with built engine
    ICudaEngine* Inference::createCudaEngine(std::string const& onnxModelPath) {
        // See link sfor additional context
        constexpr auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

        std::unique_ptr<IBuilder, Destroy<IBuilder>> builder{createInferBuilder(mLogger)};
        if (!builder) throw std::runtime_error("Failed to create Infer Builder");
        mLogger.log(ILogger::Severity::kINFO, "Created Infer Builder");

        std::unique_ptr<INetworkDefinition, Destroy<INetworkDefinition>> network{builder->createNetworkV2(explicitBatch)};
        if (!network) throw std::runtime_error("Failed to create Network Definition");
        mLogger.log(ILogger::Severity::kINFO, "Created Network Definition");

        std::unique_ptr<nvonnxparser::IParser, Destroy<nvonnxparser::IParser>> parser{nvonnxparser::createParser(*network, mLogger)};
        if (!parser) throw std::runtime_error("Failed to create ONNX Parser");
        mLogger.log(ILogger::Severity::kINFO, "Created ONNX Parser");

        std::unique_ptr<IBuilderConfig, Destroy<IBuilderConfig>> config{builder->createBuilderConfig()};
        if (!config) throw std::runtime_error("Failed to create Builder Config");
        mLogger.log(ILogger::Severity::kINFO, "Created Builder Config");

        // TODO: Not needed if we already have the engine file
        //Parse the onnx from file
        if (!parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(ILogger::Severity::kINFO))) {
            throw std::runtime_error("Failed to parse ONNX file");
        }

        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1 << 30);

        // auto profile = builder->createOptimizationProfile();
        // profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMIN, Dims4{1, 3, 256, 256});
        // profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kOPT, Dims4{1, 3, 256, 256});
        // profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMAX, Dims4{32, 3, 256, 256});

        // config->addOptimizationProfile(profile);

        //Create runtime engine
        IRuntime* runtime = createInferRuntime(mLogger);

        std::filesystem::path enginePath("./tensorrt-engine-best.engine");

        //Check if engine file exists
        if (exists(enginePath)) {
            // TODO: error checking
            //Load engine from file
            std::ifstream inputFileStream("./tensorrt-engine-best.engine", std::ios::binary);
            std::stringstream engineBuffer;

            engineBuffer << inputFileStream.rdbuf();
            std::string enginePlan = engineBuffer.str();
            // TODO: deprecated
            return runtime->deserializeCudaEngine(enginePlan.data(), enginePlan.size());
        } else {
            IHostMemory* serializedEngine = builder->buildSerializedNetwork(*network, *config);
            if (!serializedEngine) throw std::runtime_error("Failed to serialize engine");

            //Create temporary engine for serializing
            ICudaEngine* tempEng = runtime->deserializeCudaEngine(serializedEngine->data(), serializedEngine->size());
            if (!tempEng) throw std::runtime_error("Failed to create temporary engine");

            //Save Engine to File
            auto trtModelStream = tempEng->serialize();
            std::ofstream outputFileStream("./tensorrt-engine-best.engine", std::ios::binary);
            outputFileStream.write(static_cast<const char*>(trtModelStream->data()), trtModelStream->size());
            outputFileStream.close();

            return tempEng;
        }
    }

    void Inference::setUpContext() {
        // Create Execution Context.
        mContext.reset(mEngine->createExecutionContext());

        mContext->setInputShape(INPUT_BINDING_NAME, mEngine->getTensorShape(INPUT_BINDING_NAME));
    }

    void Inference::doDetections(const cv::Mat& img) const{
        //Do the forward pass on the network
        launchInference(img, mOutputTensor);
        //return Parser(outputTensor).parseTensor();
    }

    cv::Mat Inference::getOutputTensor() {
        return mOutputTensor;
    }


    void Inference::launchInference(cv::Mat const& input, cv::Mat const& output) const {
        assert(!input.empty());
        assert(!output.empty());
        assert(input.isContinuous());
        assert(output.isContinuous());
        assert(mContext);
        assert(mStream);

        const int inputId = getBindingInputIndex(mContext.get());

        cudaMemcpy(mBindings[inputId], input.data, input.total() * input.elemSize(), cudaMemcpyHostToDevice);
        mContext->executeV2(mBindings.data());
        cudaMemcpy(output.data, mBindings[1 - inputId], output.total() * output.elemSize(), cudaMemcpyDeviceToHost);
    }


    /**
* Takes tensor bindings and allocates memory on the GPU for input and output tensors
* Requires enginePtr, bindings, inputTensor, and outputTensor
* Modifies bindings, inputTensor, and outputTensor
*/
    void Inference::prepTensors() {

        for (int i = 0; i < mEngine->getNbIOTensors(); i++) {
            const char* tensorName = mEngine->getIOTensorName(i);

            auto [rank, extents] = mEngine->getTensorShape(tensorName);

            // Multiply sizeof(float) by the product of the extents
            // This is essentially: element count * size of each element
            const std::size_t size = std::reduce(extents, extents + rank, sizeof(float), std::multiplies<>());
            // Create GPU memory for TensorRT to operate on
            cudaMalloc(mBindings.data() + i, size);
        }

        // TODO(quintin): Avoid hardcoding this
        assert(mContext);
        const Dims outputTensorDims = mEngine->getTensorShape(OUTPUT_BINDING_NAME);
        for (int i = 0; i < outputTensorDims.nbDims; i++) {
            char message[512];
            std::snprintf(message, sizeof(message), "size %d %d", i, outputTensorDims.d[i]);
            mLogger.log(nvinfer1::ILogger::Severity::kINFO, message);
        }

        mOutputTensor = cv::Mat::zeros(outputTensorDims.d[1], outputTensorDims.d[2], CV_MAKE_TYPE(CV_32F, 1));
    }

    int Inference::getBindingInputIndex(const IExecutionContext* context) {
        return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
    }
} // namespace mrover
