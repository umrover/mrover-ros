#include <streaming.hpp>

#include <format>
#include <iostream>

#include <opencv2/videoio.hpp>

int main() {
    Streamer streamer{{640, 480}};

    cv::VideoCapture cap{
            0,
            cv::CAP_V4L2,
            {cv::CAP_PROP_FRAME_WIDTH, 640,
             cv::CAP_PROP_FRAME_HEIGHT, 480,
             cv::CAP_PROP_CONVERT_RGB, false,
             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2')},
    };
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (cap.read(frame)) {
        streamer.feed(frame);
    }

    return 0;
}