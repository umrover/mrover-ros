#include "object_detector.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <string>
#include <vector>

namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        std::vector<std::string> classes = {
                "person",
                "bicycle",
                "car",
                "motorcycle",
                "airplane",
                "bus",
                "train",
                "truck",
                "boat",
                "traffic light",
                "fire hydrant",
                "stop sign",
                "parking meter",
                "bench",
                "bird",
                "cat",
                "dog",
                "horse",
                "sheep",
                "cow",
                "elephant",
                "bear",
                "zebra",
                "giraffe",
                "backpack",
                "umbrella",
                "handbag",
                "tie",
                "suitcase",
                "frisbee",
                "skis",
                "snowboard",
                "sports ball",
                "kite",
                "baseball bat",
                "baseball glove",
                "skateboard",
                "surfboard",
                "tennis racket",
                "bottle",
                "wine glass",
                "cup",
                "fork",
                "knife",
                "spoon",
                "bowl",
                "banana",
                "apple",
                "sandwich",
                "orange",
                "broccoli",
                "carrot",
                "hot dog",
                "pizza",
                "donut",
                "cake",
                "chair",
                "couch",
                "potted plant",
                "bed",
                "dining table",
                "toilet",
                "tv",
                "laptop",
                "mouse",
                "remote",
                "keyboard",
                "cell phone",
                "microwave",
                "oven",
                "toaster",
                "sink",
                "refrigerator",
                "book",
                "clock",
                "vase",
                "scissors",
                "teddy bear",
                "hair drier",
                "toothbrush"};

        cv::Mat imageView{static_cast<int>(msg->width), static_cast<int>(msg->height), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        cv::Mat imageBlob = cv::dnn::blobFromImage(imageView);
        //Set the input data for the model
        mNet.setInput(imageBlob);

        //Run the network and get the results
        std::vector<cv::Mat> outputsMats;
        mNet.forward(outputsMats, mNet.getUnconnectedOutLayersNames());

        //Result Variables
        cv::Mat mat1 = outputsMats[0];
        float classConfidence = 0.0;
        cv::Rect box;

        //Get the Data from the MAT
        auto data = (float*) mat1.data; //Why does it want me to use auto here?

        //Get the confidence
        classConfidence = data[4];

        //Get the bounding box
        box.x = (int) data[0];
        box.y = (int) data[1];
        box.width = (int) data[2];
        box.height = (int) data[3];

        //Get the ID of the object and its score
        float* dataBegin = data + 5;                                       //Start at the beginning of the scores
        cv::Mat classScores(1, (int) classes.size(), CV_32FC1, dataBegin); //Make a new mat for minMaxLoc
        cv::Point classID;                                                 //A place for the desired class id to go
        double classScore;                                                 //The corresponding score
        cv::minMaxLoc(classScore, nullptr, &classScore, nullptr, &classID);


        //Look at yolov8 documentation for the output matrix

        /*
         * TODO(percep/obj-detector):
         * 0. Google "OpenCV DNN example." View a couple of tutorials. Most will be in Python but the C++ API is similar.
         * 1. Use cv::dnn::blobFromImage to convert the image to a blob
         * 2. Use mNet.forward to run the network
         * 3. Parse the output of the network to get the bounding boxes
         * 4. Publish the bounding boxes
         */
    }

} // namespace mrover
