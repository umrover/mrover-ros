#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

struct Logger {

    void log(nvinfer1::ILogger::Severity severity, nvinfer1::AsciiChar const* msg) {
    }
};

Logger logger;

void setup() {
    // TOOD: fix this
    nvinfer1::createInferBuilder(logger);
}