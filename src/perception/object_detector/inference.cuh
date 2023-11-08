// Cpp native
#pragma once
#include <NvInferRuntime.h>
#include <fstream>
#include <iterator>
#include <memory>

#include <random>
#include <string>
#include <vector>


//Tensor-RT Specific
#include "cudaWrapper.cuh"
#include <NvInfer.h>
#include <NvOnnxParser.h>


#include "ioHelper.cuh"

// OpenCV / DNN / Inference
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

//MROVER MADE FILES
#include "object_detector.hpp"
#include "pch.hpp"

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

// ROS Mesage -> CV View Matrix -> cv::blobFromImage -> cv::Mat (input)
// Preallocated CV::Mat of floats of correct size -> transpose into CV::Mat (also preallocated

namespace mrover {
    class Inference {
    private:
        static const int BATCH_SIZE = 1;

        nvinfer1::Logger logger;


        //Ptr to the engine
        std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> enginePtr;

        //Ptr to the context
        std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> contextPtr;

        //Input, output and reference tensors
        cv::Mat inputTensor;
        cv::Mat outputTensor;
        //Num Entries
        nvinfer1::Dims3 inputEntries;
        nvinfer1::Dims3 outputEntries;

        //Cuda Stream
        cudawrapper::CudaStream stream;

        //Bindings
        std::array<void*, 2> bindings{};

        //ONNX Model Path
        std::string onnxModelPath;

        //Size of Model
        cv::Size modelInputShape;
        cv::Size modelOutputShape;

    private:
        //STATIC FUNCTIONS
        static int getBindingInputIndex(nvinfer1::IExecutionContext* context);

    public:
        Inference(std::string onnxModelPath, cv::Size modelInputShape, std::string classesTxtFile);


        std::vector<Detection> runInference(cv::Mat const& input);

    private:
        //Creates a ptr to the engine
        ICudaEngine* createCudaEngine(std::string& onnxModelPath);

        void launchInference(void* input, void* output);

        void setUpContext(const std::string& inputFile); //This will need to be adjusted to the img msg, but I think its ok to just run pictures as an intial test

        void prepTensors();

        void setUpContext();

    public:
        void doDetections(cv::Mat& img);
    };
} // namespace mrover
