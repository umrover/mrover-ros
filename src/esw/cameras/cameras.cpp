#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <thread>
#include <iostream>

#include <opencv2/videoio.hpp>

int main() {
    Streamer streamer{"127.0.0.1", 8080};

    Encoder encoder{{640, 480}};

    //    cv::VideoCapture cap{
    //            0,
    //            cv::CAP_V4L2,
    //            {cv::CAP_PROP_FRAME_WIDTH, 640,
    //             cv::CAP_PROP_FRAME_HEIGHT, 480,
    //             cv::CAP_PROP_CONVERT_RGB, false,
    //             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2')},
    //    };
    //    if (!cap.isOpened()) {
    //        std::cerr << "Failed to open camera" << std::endl;
    //        return 1;
    //    }
    //
    //    cv::Mat frame;
    //    while (cap.read(frame)) {
    //        encoder.feed(frame);
    //    }

    while (streamer.m_client->is_open()) {
        cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
        Encoder::BitstreamView view = encoder.feed(frame);
        std::span<std::byte> data{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        streamer.feed(data);
        std::this_thread::sleep_for(std::chrono::milliseconds{1000 / 30});
    }

    return 0;
}