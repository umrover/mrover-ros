#pragma once

#include "pch.hpp"

#include "logger.cuh"

#include <NvInfer.h>

using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;


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

    /**
     * @brief Creates the Cuda engine which is machine specific
     *
     * Uses the file path to locate the onnx model and the modelName to locate the engine file
     */
    auto createCudaEngine(std::filesystem::path const& onnxModelPath, std::string const& modelName) -> ICudaEngine*;

    /**
     * @brief Performs the Model forward pass and allocates the reults in the output tensor
     *
     * Takes the input Mat in the format for CNN and requires a SIZED empty Mat to store the output
     */
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
    explicit Inference(std::filesystem::path const& onnxModelPath, std::string const& modelName);

    /**
     * @brief Runs the forward pass on the given input image in CNN format
     */
    auto doDetections(cv::Mat const& img) const -> void;

    /**
     * @brief Retrieves the mat where the output from the forward pass was stored
     */
    auto getOutputTensor() -> cv::Mat;
};
