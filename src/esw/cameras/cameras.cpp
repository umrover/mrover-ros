#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <iostream>
#include <thread>

#include <opencv2/videoio.hpp>

int main() {
    Streamer streamer{"127.0.0.1", 8080};

    Encoder encoder{{320, 240}};

    cv::VideoCapture cap{
            0,
            cv::CAP_V4L2,
            {cv::CAP_PROP_FRAME_WIDTH, 320,
             cv::CAP_PROP_FRAME_HEIGHT, 240,
             cv::CAP_PROP_FPS, 60,
             cv::CAP_PROP_CONVERT_RGB, false,
             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2')},
    };
    if (!cap.isOpened()) {
        throw std::runtime_error{"Failed to open camera"};
    }
    //
    //    cv::Mat frame;
    //    while (cap.read(frame)) {
    //        encoder.feed(frame);
    //    }

    cv::Mat frame;
    while (streamer.m_client->is_open() && cap.read(frame)) {
        Encoder::BitstreamView view = encoder.feed(frame);
        std::span<std::byte> data{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        std::cout << std::format("Sending {} bytes\n", data.size());
        streamer.feed(data);
        //        std::this_thread::sleep_for(std::chrono::milliseconds{1000 / 30});
    }

    return EXIT_SUCCESS;
}