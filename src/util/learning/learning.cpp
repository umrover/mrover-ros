#include "learning.hpp"

using namespace std;

Learning::Learning() = default;

Learning::Learning(string& modelName) : mModelName{std::move(modelName)} {

    std::filesystem::path packagePath = ros::package::getPath("mrover");
    std::filesystem::path modelFileName = mModelName + ".onnx";
    std::filesystem::path modelPath = packagePath / "data" / modelFileName;

    mInferenceWrapper = InferenceWrapper{modelPath, mModelName};
}

Learning::~Learning() = default;

auto Learning::modelForwardPass(cv::Mat const& blob, std::vector<Detection>& detections, float modelScoreThreshold, float modelNMSThreshold) const -> void {
    mInferenceWrapper.doDetections(blob);
    cv::Mat output = mInferenceWrapper.getOutputTensor();
    parseModelOutput(output, detections, modelScoreThreshold, modelNMSThreshold);
}

auto Learning::parseModelOutput(cv::Mat& output, std::vector<Detection>& detections, float modelScoreThreshold, float modelNMSThreshold) const -> void {
    // Parse model specific dimensioning from the output

    // The input to this function is expecting a YOLOv8 style model, thus the dimensions should be > rows
    if (output.cols <= output.rows) {
        throw std::runtime_error("Something is wrong Model with interpretation");
    }

    int rows = output.cols;
    int dimensions = output.rows;

    // The output of the model is a batchSizex84x8400 matrix
    // This converts the model to a TODO: Check this dimensioning
    output = output.reshape(1, dimensions);
    cv::transpose(output, output);

    // This function expects the image to already be in the correct format thus no distrotion is needed
    float const xFactor = 1.0;
    float const yFactor = 1.0;

    // Intermediate Storage Containers
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Each row of the output is a detection with a bounding box and associated class scores
    for (int r = 0; r < rows; ++r) {
        // Skip first four values as they are the box data
        cv::Mat scores = output.row(r).colRange(4, dimensions);

        cv::Point classId;
        double maxClassScore;
        cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &classId);

        // Determine if the class is an acceptable confidence level, otherwise disregard
        if (maxClassScore <= modelScoreThreshold) continue;

        confidences.push_back(static_cast<float>(maxClassScore));
        classIds.push_back(classId.x);

        // Get the bounding box data
        cv::Mat box = output.row(r).colRange(0, 4);
        auto x = box.at<float>(0);
        auto y = box.at<float>(1);
        auto w = box.at<float>(2);
        auto h = box.at<float>(3);

        // Cast the corners into integers to be used on pixels
        auto left = static_cast<int>((x - 0.5 * w) * xFactor);
        auto top = static_cast<int>((y - 0.5 * h) * yFactor);
        auto width = static_cast<int>(w * xFactor);
        auto height = static_cast<int>(h * yFactor);

        // Push abck the box into storage
        boxes.emplace_back(left, top, width, height);
    }

    //Coalesce the boxes into a smaller number of distinct boxes
    std::vector<int> nmsResult;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nmsResult);

    // Fill in the output Detections Vector
    for (int i: nmsResult) {
        Detection result;

        //Fill in the id and confidence for the class
        result.classId = classIds[i];
        result.confidence = confidences[i];

        //Fill in the class name and box information
        result.className = classes[result.classId];
        result.box = boxes[i];

        //Push back the detection into the for storagevector
        detections.push_back(result);
    }
}
