#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <NvInferLegacyDims.h>
#include <NvInferRuntimeBase.h>
#include <fstream>
#include <iterator>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <random>
#include <string>
#include <vector>


//Tensor-RT Specific
#include "cudaWrapper.h"
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvOnnxParser.h>


#include "ioHelper.h"

// OpenCV / DNN / Inference
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "pch.hpp"

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

// ROS Mesage -> CV View Matrix -> cv::blobFromImage -> cv::Mat (input)
// Preallocated CV::Mat of floats of correct size -> transpose into CV::Mat (also preallocated

struct Detection {
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

class Inference {
private:
    static const int BATCH_SIZE = 1;

    Logger logger;


    //Ptr to the engine
    std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> enginePtr;

    //Ptr to the context
    std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> context;

    //Input, output and reference tensors
    cv::Mat inputTensor;
    cv::Mat outputTensor;
    //Num Entries
    nvinfer1::Dims3 inputEntries;
    nvinfer1::Dims3 outputEntries;

    //Cuda Stream
    CudaStream stream;

    //Bindings
    std::array<void*, 2> bindings{};

    //ONNX Model Path
    std::string onnxModelPath;

    //Size of Model
    std::modelInputShape;
    cv::Size modelOutputShape;

private:
    //STATIC FUNCTIONS
    static int getBindingInputIndex(nvinfer1::IExecutionContext* context);

public:
    Inference() = default;

    Inference(std::string_view onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string_view classesTxtFile = "");


    std::vector<Detection> runInference(cv::Mat const& input);

private:
    //Creates a ptr to the engine
    void createCudaEngine(std::string& onnxModelPath);

    void Inference::launchInference(float* input, float* output, int tensorSize);

    void setUpContext(const std::string& inputFile); //This will need to be adjusted to the img msg, but I think its ok to just run pictures as an intial test

    void prepTensors();

    void Inference::setUpContext();

    std::vector<Detection> Inference::doDetections(const std::string& inputFile);
}
#endif // INFERENCE_H
