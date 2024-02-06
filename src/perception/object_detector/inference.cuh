#pragma once

#include "logger.cuh"
#include "pch.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cstdio>
#include <cuda_runtime_api.h>
#include <opencv4/opencv2/core/hal/interface.h>
#include <string>

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

namespace mrover {

    class Inference {
    private:
        //Model Path
        std::string mModelPath;

        //Init Logger
        nvinfer1::Logger mLogger;

        //Ptr to the engine
        std::unique_ptr<ICudaEngine> mEngine{};

        //Ptr to the context
        std::unique_ptr<IExecutionContext> mContext{};

        //Input, output and reference tensors
        cv::Mat mInputTensor;
        cv::Mat mOutputTensor;

        //Bindings
        std::array<void*, 2> mBindings{};

        //Size of Model
        cv::Size mModelInputShape;
        cv::Size mModelOutputShape;

        //STATIC FUNCTIONS
        static auto getBindingInputIndex(IExecutionContext const* context) -> int;

        //Creates a ptr to the engine
        auto createCudaEngine(std::filesystem::path const& onnxModelPath) -> ICudaEngine*;

        //Launch the model execution onto the GPU
        void launchInference(cv::Mat const& input, cv::Mat const& output) const;

        /**
         * @brief Prepares the tensors for inference.
         * 
         * Takes tensor bindings and allocates memory on the GPU for input and output tensors
         * Requires enginePtr, bindings, inputTensor, and outputTensor
         * 
         * Requires enginePtr, bindings, inputTensor, and outputTensor
         * Modifies bindings, inputTensor, and outputTensor
         */
        void prepTensors();

        /**
        * @brief Creates the execution context for the model
        */
        void createExecutionContext();

    public:
        //Inference Constructor
        Inference(std::filesystem::path const& onnxModelPath);

        //Forward Pass of the model
        void doDetections(cv::Mat const& img) const;

        //Get the output tensor with in a YOLO v8 style data structure
        cv::Mat getOutputTensor();
    };

} // namespace mrover
