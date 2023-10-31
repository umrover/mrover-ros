#include "inference.h"


using namespace nvinfer1;
using namespace std;

InferenceNew::InferenceNew(std::string_view onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string_view classesTxtFile = "") {
    createCudaEngine(onnxModelPath, BATCH_SIZE);

    assert(InferenceNew::enginePtr->getNbBindings() == 2);
    assert(InferenceNew::enginePtr->bindingIsInput(0) ^ enginePtr->bindingIsInput(1));

    InferenceNew::onnxModelPath = onnxModelPath;

    InferenceNew::prepTensors();
}

void InferenceNew::createCudaEngine(std::string_view onnxModelPath, int batchSize) {
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<nvinfer1::IBuilder, Destroy<nvinfer1::IBuilder>> builder{nvinfer1::createInferBuilder(gLogger)};
    std::unique_ptr<nvinfer1::INetworkDefinition, Destroy<nvinfer1::INetworkDefinition>> network{builder->createNetworkV2(explicitBatch)};
    std::unique_ptr<nvonnxparser::IParser, Destroy<nvonnxparser::IParser>> parser{nvonnxparser::createParser(*network, gLogger)};
    std::unique_ptr<nvinfer1::IBuilderConfig, Destroy<nvinfer1::IBuilderConfig>> config{builder->createBuilderConfig()};

    if (!parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(ILogger::Severity::kINFO))) {
        cout << "ERROR: could not parse input engine." << endl;
    }

    builder->setMaxBatchSize(batchSize);
    config->setMaxWorkspaceSize((1 << 30));

    auto profile = builder->createOptimizationProfile();
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMIN, Dims4{1, 3, 256, 256});
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kOPT, Dims4{1, 3, 256, 256});
    profile->setDimensions(network->getInput(0)->getName(), OptProfileSelector::kMAX, Dims4{32, 3, 256, 256});
    config->addOptimizationProfile(profile);

    InferenceNew::enginePtr = builder->buildEngineWithConfig(*network, *config);
}

static int InferenceNew::getBindingInputIndex(nvinfer1::IExecutionContext* context) {
    return !context->getEngine().bindingIsInput(0); // 0 (false) if bindingIsInput(0), 1 (true) otherwise
}