#include "inference.h"
#include <string>


using namespace nvinfer1;
using namespace std;

//  : logger{}, inputTensor{}, outputTensor{}, referenceTensor{}, stream{}
Inference::Inference(std::string_view onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string_view classesTxtFile = "") : logger{}, inputTensor{}, outputTensor{}, referenceTensor{}, stream{} {

    createCudaEngine(onnxModelPath, Inference::BATCH_SIZE);

    assert(this->enginePtr->getNbBindings() == 2);
    assert(this->enginePtr->bindingIsInput(0) ^ enginePtr->bindingIsInput(1));

    this->onnxModelPath = onnxModelPath;
}