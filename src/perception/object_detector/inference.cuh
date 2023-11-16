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
        constexpr static int BATCH_SIZE = 1;

        nvinfer1::Logger mLogger;

        //Ptr to the engine
        std::unique_ptr<ICudaEngine, nvinfer1::Destroy<ICudaEngine>> mEngine;

        //Ptr to the context
        std::unique_ptr<IExecutionContext, nvinfer1::Destroy<IExecutionContext>> mContext;

        //Input, output and reference tensors
        cv::Mat mInputTensor;
        cv::Mat mOutputTensor;
        //Num Entries
        nvinfer1::Dims3 mInputDimensions;
        nvinfer1::Dims3 mOutputDimensions;

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

        void launchInference(void* input, void* output);

        void prepTensors();

        void setUpContext();

    public:
        void doDetections(cv::Mat& img);
    };

} // namespace mrover
