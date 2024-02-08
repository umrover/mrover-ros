#pragma once

#include "pch.hpp"

#include "logger.cuh"

#include <NvInfer.h>

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;

namespace mrover {

    class Inference {
        std::string mModelPath;

        nvinfer1::Logger mLogger;

        std::unique_ptr<ICudaEngine> mEngine{};

        std::unique_ptr<IExecutionContext> mContext{};

        cv::Mat mInputTensor;
        cv::Mat mOutputTensor;

        std::array<void*, 2> mBindings{};

        cv::Size mModelInputShape;
        cv::Size mModelOutputShape;

        static auto getBindingInputIndex(IExecutionContext const* context) -> int;

        auto createCudaEngine(std::filesystem::path const& onnxModelPath) -> ICudaEngine*;

        auto launchInference(cv::Mat const& input, cv::Mat const& output) const -> void;

        /**
         * @brief Prepares the tensors for inference.
         * 
         * Takes tensor bindings and allocates memory on the GPU for input and output tensors
         * Requires enginePtr, bindings, inputTensor, and outputTensor
         * 
         * Requires enginePtr, bindings, inputTensor, and outputTensor
         * Modifies bindings, inputTensor, and outputTensor
         */
        auto prepTensors() -> void;

        /**
        * @brief Creates the execution context for the model
        */
        auto createExecutionContext() -> void;

    public:
        explicit Inference(std::filesystem::path const& onnxModelPath);

        auto doDetections(cv::Mat const& img) const -> void;

        auto getOutputTensor() -> cv::Mat;
    };

} // namespace mrover