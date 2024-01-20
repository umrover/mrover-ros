// Cpp native
#pragma once

#include "pch.hpp"

#include <NvInfer.h>

#include "logger.cuh"

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

namespace mrover {

    class Inference {
        //Init Logger
        nvinfer1::Logger mLogger;

        //Ptr to the engine
        std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> mEngine{};

        //Ptr to the context
        std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> mContext{};

        //Input, output and reference tensors
        cv::Mat mInputTensor;
        cv::Mat mOutputTensor;

        //Bindings
        std::array<void*, 2> mBindings{};

        //Size of Model
        cv::Size mModelInputShape;
        cv::Size mModelOutputShape;

        //STATIC FUNCTIONS
        static int getBindingInputIndex(const IExecutionContext* context);

    private:
        //Creates a ptr to the engine
        ICudaEngine* createCudaEngine(std::filesystem::path const& onnxModelPath);

        //Launch the model execution onto the GPU
        void launchInference(cv::Mat const& input, cv::Mat const& output) const;

        //Init the tensors
        void prepTensors();

        //Init the execution context
        void setUpContext();

    public:
        //Inference Constructor
        Inference(std::filesystem::path const& onnxModelPath);

        //Forward Pass of the model
        void doDetections(const cv::Mat& img) const;

        //Get the output tensor with in a YOLO v8 style data structure 
        cv::Mat getOutputTensor();
    };

} // namespace mrover
