#include <opencv2/core/mat.hpp>
#include <string>


namespace mrover {


    /*

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

std::vector<Detection> Inference::doDetections(cv::Mat& img) {
    //Do the forward pass on the network
    launchInference(img.data, this->outputTensor.data, );

    return Parser(this->outputTensor).parseTensor();
}
*/

    /*
void Inference::setUpContext() {
    // Create Execution Context.
    this->context.reset(this->enginePtr->createExecutionContext());

    Dims dims_i{this->enginePtr->getBindingDimensions(0)};
    Dims4 inputDims{Inference::BATCH_SIZE, dims_i.d[1], dims_i.d[2], dims_i.d[3]};
    context->setBindingDimensions(0, inputDims);
}
*/

} // namespace mrover