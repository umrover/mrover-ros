#include <streaming.hpp>

#include <opencv2/videoio.hpp>

int main() {
    Streamer streamer{{640, 480}};

    cv::VideoCapture cap{0};
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