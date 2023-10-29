#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <fstream>
#include <memory>
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

using nvinfer1::cuda;
using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

struct Detection {
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

class Inference {
public:
    Inference() = default;
    Inference(const std::string& onnxModelPath, const cv::Size& modelInputShape = {640, 640}, const std::string& classesTxtFile = "", const bool& runWithCuda = true);
    std::vector<Detection> runInference(const cv::Mat& input);

private:
    void loadClassesFromFile();
    void loadOnnxNetwork();
    cv::Mat formatToSquare(const cv::Mat& source);

    std::string modelPath;
    std::string classesPath;
    bool cudaEnabled{};

    std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

    cv::Size2f modelShape{};

    float modelConfidenceThreshold{0.25};
    float modelScoreThreshold{0.45};
    float modelNMSThreshold{0.50};

    bool letterBoxForSquare = true;

    cv::dnn::Net net;
};

class InferenceNew {
private:
    static const int BATCH_SIZE = 1;

    Logger logger;


    //Ptr to the engine
    std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> enginePtr;
    //Ptr to the context
    std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> context;
    //Input, output and reference tensors
    std::vector<float> inputTensor;
    std::vector<float> outputTensor;
    std::vector<float> referenceTensor;

    //Cuda Stream
    CudaStream stream;

    //Bindings
    std::array<void*, 2> bindings{};

public:
    InferenceNew() = default;

    InferenceNew(std::string_view onnxModelPath, cv::Size modelInputShape = {640, 640}, std::string_view classesTxtFile = "");

    std::vector<Detection> runInference(cv::Mat const& input);

private:
    //Creates a ptr to the engine
    void createCudaEngine(std::string_view onnxModelPath, int batchSize);

    void launchInference(IExecutionContext* context, cudaStream_t stream, std::vector<float> const& inputTensor, std::vector<float>& outputTensor, void** bindings, int batchSize);
}
#endif // INFERENCE_H
