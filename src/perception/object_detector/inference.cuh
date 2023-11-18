// Cpp native
#pragma once

#include "pch.hpp"

#include <NvInfer.h>

#include "cudaWrapper.cuh"
#include "ioHelper.cuh"

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

// ROS Mesage -> CV View Matrix -> cv::blobFromImage -> cv::Mat (input)
// Preallocated CV::Mat of floats of correct size -> transpose into CV::Mat (also preallocated

namespace mrover {

    class Inference {
        nvinfer1::Logger mLogger;

        //Ptr to the engine
        std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> mEngine;

        //Ptr to the context
        std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> mContext;

        //Input, output and reference tensors
        cv::Mat mInputTensor;
        cv::Mat mOutputTensor;

        //Cuda Stream
        std::optional<cudawrapper::CudaStream> mStream;

        //Bindings
        std::array<void*, 2> mBindings{};

        //ONNX Model Path
        std::string mOnnxModelPath;

        //Size of Model
        cv::Size mModelInputShape;
        cv::Size mModelOutputShape;

        //STATIC FUNCTIONS
        static int getBindingInputIndex(IExecutionContext* context);

    public:
        Inference(std::string const& onnxModelPath, cv::Size modelInputShape, std::string const& classesTxtFile);

    private:
        //Creates a ptr to the engine
        ICudaEngine* createCudaEngine(std::string const& onnxModelPath);

        void launchInference(cv::Mat const& input, cv::Mat const& output);

        void prepTensors();

        void setUpContext();

    public:
        void doDetections(const cv::Mat& img);

        cv::Mat getOutputTensor();
    };

} // namespace mrover
