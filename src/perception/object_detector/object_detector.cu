#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "ioHelper.h"

#include <memory>
#include <string.h>

#include "inference.h"

#include <vector>

using namespace nvinfer1;


// struct Logger {

//     void log(nvinfer1::ILogger::Severity severity, nvinfer1::AsciiChar const* msg) {
//     }
// };


// static Logger logger;

// __global__ void forward() {
// }


// void setup() {
//     // TOOD: fix this
//     nvinfer1::createInferBuilder(logger);
// }

void InferenceNew::launchInference(IExecutionContext* context, cudaStream_t stream, std::vector<float> const& inputTensor, std::vector<float>& outputTensor, void** bindings, int batchSize) {
    int inputId = InferenceNew::getBindingInputIndex(context);

    cudaMemcpyAsync(bindings[inputId], inputTensor.data(), inputTensor.size() * sizeof(float), cudaMemcpyHostToDevice, stream);

    context->enqueueV3(stream);

    cudaMemcpyAsync(outputTensor.data(), bindings[1 - inputId], outputTensor.size() * sizeof(float), cudaMemcpyDeviceToHost, stream);
}
