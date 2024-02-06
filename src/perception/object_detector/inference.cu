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

    //Create names for common model names
    constexpr static char const* INPUT_BINDING_NAME = "images";
    constexpr static char const* OUTPUT_BINDING_NAME = "output0";

    Inference::Inference(std::filesystem::path const& onnxModelPath) {
        //Create the engine object from either the file or from onnx file
        mEngine = std::unique_ptr<ICudaEngine>{createCudaEngine(onnxModelPath)};
        if (!mEngine) throw std::runtime_error("Failed to create CUDA engine");

        mLogger.log(ILogger::Severity::kINFO, "Created CUDA Engine");

        //Check some assumptions about the model
        if (mEngine->getNbIOTensors() != 2) throw std::runtime_error("Invalid Binding Count");
        if (mEngine->getTensorIOMode(INPUT_BINDING_NAME) != TensorIOMode::kINPUT) throw std::runtime_error("Expected Input Binding 0 Is An Input");
        if (mEngine->getTensorIOMode(OUTPUT_BINDING_NAME) != TensorIOMode::kOUTPUT) throw std::runtime_error("Expected Input Binding Input To Be 1");

        createExecutionContext();

        //Init the io tensors on the GPU
        prepTensors();
    }

    ICudaEngine* Inference::createCudaEngine(std::filesystem::path const& onnxModelPath) {

        mModelPath = {onnxModelPath.c_str()};

        //Define the size of Batches
        constexpr auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

        //Init logger
        std::unique_ptr<IBuilder> builder{createInferBuilder(mLogger)};
        if (!builder) throw std::runtime_error("Failed to create Infer Builder");
        mLogger.log(ILogger::Severity::kINFO, "Created Infer Builder");

        //Init Network
        std::unique_ptr<INetworkDefinition> network{builder->createNetworkV2(explicitBatch)};
        if (!network) throw std::runtime_error("Failed to create Network Definition");
        mLogger.log(ILogger::Severity::kINFO, "Created Network Definition");

        //Init the onnx file parser
        std::unique_ptr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, mLogger)};
        if (!parser) throw std::runtime_error("Failed to create ONNX Parser");
        mLogger.log(ILogger::Severity::kINFO, "Created ONNX Parser");

        //Init the builder
        std::unique_ptr<IBuilderConfig> config{builder->createBuilderConfig()};
        if (!config) throw std::runtime_error("Failed to create Builder Config");
        mLogger.log(ILogger::Severity::kINFO, "Created Builder Config");

        //Parse the onnx from file
        if (!parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(ILogger::Severity::kINFO))) {
            throw std::runtime_error("Failed to parse ONNX file");
        }

        //Create runtime engine
        IRuntime* runtime = createInferRuntime(mLogger);

        //Define the engine file location relative to the mrover package
        std::filesystem::path packagePath = ros::package::getPath("mrover");
        std::filesystem::path enginePath = packagePath / "data" / std::string("tensorrt-engine-").append(mModelPath).append(".engine");

        //Check if engine file exists
        if (!exists(enginePath)) {
            ROS_WARN_STREAM("Optimizing ONXX model for TensorRT. This make take a long time...");

            //Create the Engine from onnx file
            IHostMemory* serializedEngine = builder->buildSerializedNetwork(*network, *config);
            if (!serializedEngine) throw std::runtime_error("Failed to serialize engine");

            //Create temporary engine for serializing
            ICudaEngine* tempEng = runtime->deserializeCudaEngine(serializedEngine->data(), serializedEngine->size());
            if (!tempEng) throw std::runtime_error("Failed to create temporary engine");

            //Save Engine to File
            auto trtModelStream = tempEng->serialize();
            std::ofstream outputFileStream(enginePath, std::ios::binary);
            outputFileStream.write(static_cast<char const*>(trtModelStream->data()), trtModelStream->size());
            outputFileStream.close();

            return tempEng;
        }

        //Load engine from file
        std::ifstream inputFileStream(enginePath, std::ios::binary);
        std::stringstream engineBuffer;

        //Stream in the engine file to the buffer
        engineBuffer << inputFileStream.rdbuf();
        std::string enginePlan = engineBuffer.str();
        //Deserialize the Cuda Engine file from the buffer
        return runtime->deserializeCudaEngine(enginePlan.data(), enginePlan.size());
    }

    void Inference::createExecutionContext() {
        // Create Execution Context.
        mContext.reset(mEngine->createExecutionContext());

        //Set up the input tensor sizing
        mContext->setInputShape(INPUT_BINDING_NAME, mEngine->getTensorShape(INPUT_BINDING_NAME));
    }

    void Inference::doDetections(cv::Mat const& img) const {
        //Do the forward pass on the network
        launchInference(img, mOutputTensor);
    }

    cv::Mat Inference::getOutputTensor() {
        //Returns the output tensor
        return mOutputTensor;
    }


    void Inference::launchInference(cv::Mat const& input, cv::Mat const& output) const {
        //Assert these items have been initialized
        assert(!input.empty());
        assert(!output.empty());
        assert(input.isContinuous());
        assert(output.isContinuous());
        assert(mContext);

        //Get the binding id for the input tensor
        int const inputId = getBindingInputIndex(mContext.get());

        //Memcpy the input tensor from the host to the gpu
        cudaMemcpy(mBindings[inputId], input.data, input.total() * input.elemSize(), cudaMemcpyHostToDevice);

        //Execute the model on the gpu
        mContext->executeV2(mBindings.data());

        //Memcpy the output tensor from the gpu to the host
        cudaMemcpy(output.data, mBindings[1 - inputId], output.total() * output.elemSize(), cudaMemcpyDeviceToHost);
    }


    void Inference::prepTensors() {
        //Assign the properties to the input and output tensors
        for (int i = 0; i < mEngine->getNbIOTensors(); i++) {
            char const* tensorName = mEngine->getIOTensorName(i);
            auto [rank, extents] = mEngine->getTensorShape(tensorName);

            // Multiply sizeof(float) by the product of the extents
            // This is essentially: element count * size of each element
            std::size_t const size = std::reduce(extents, extents + rank, sizeof(float), std::multiplies<>());
            // Create GPU memory for TensorRT to operate on
            cudaMalloc(mBindings.data() + i, size);
        }

        assert(mContext);
        //Create an appropriately sized output tensor
        Dims const outputTensorDims = mEngine->getTensorShape(OUTPUT_BINDING_NAME);
        for (int i = 0; i < outputTensorDims.nbDims; i++) {
            char message[512];
            std::snprintf(message, sizeof(message), "size %d %d", i, outputTensorDims.d[i]);
            mLogger.log(nvinfer1::ILogger::Severity::kINFO, message);
        }

        //Create the mat wrapper around the output matrix for ease of use
        mOutputTensor = cv::Mat::zeros(outputTensorDims.d[1], outputTensorDims.d[2], CV_MAKE_TYPE(CV_32F, 1));
    }

    int Inference::getBindingInputIndex(IExecutionContext const* context) {
        //Returns the id for the input tensor
        return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
    }
} // namespace mrover
