#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <iostream>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

//#include <libde265/de265.h>

int main() {
    StreamServer streamServer{"0.0.0.0", 8080};

    cv::Size size{1280, 720};

    NvEncoder nvEncoder{size};

    cv::VideoCapture cap{std::format("v4l2src ! videoconvert ! video/x-raw,width={},height={},format=I420,framerate=10/1 ! appsink", size.width, size.height), cv::CAP_GSTREAMER};

    //    cv::VideoCapture cap{std::format("videotestsrc ! video/x-raw,width={},height={},format=I420,framerate=30/1 ! appsink", size.width, size.height), cv::CAP_GSTREAMER};

    if (!cap.isOpened()) throw std::runtime_error{"Failed to open capture"};

    size_t totalSent = 0;
    size_t totalFrames = 0;
    auto now = std::chrono::high_resolution_clock::now();

    cv::Mat frameI420;
    while (streamServer.m_client->is_open() && cap.read(frameI420)) {
        assert(frameI420.type() == CV_8UC1);

        cv::imshow("USB", frameI420);
        cv::waitKey(1);

        NvEncoder::BitstreamView view = nvEncoder.feed(frameI420);
        std::span span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        streamServer.feed(span);

        totalSent += span.size_bytes();
        totalFrames++;
        double MB = totalSent / 1024.0 / 1024.0;
        auto elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - now);
        double MBps = MB / elapsed.count();
        double fps = totalFrames / elapsed.count();

        std::cout << std::format("MBps: {} FPS: {}\n", MBps, fps);
    }

    //    static de265_decoder_context* decoder = de265_new_decoder();
    //    if (!decoder) return EXIT_FAILURE;
    //    if (de265_error error = de265_start_worker_threads(decoder, 0); error != DE265_OK) throw std::runtime_error{"Errored starting worker threads"};
    //
    //    int more = 1;
    //    while (more) {
    //        de265_error status = de265_decode(decoder, &more);
    //
    //        if (status == DE265_ERROR_WAITING_FOR_INPUT_DATA) {
    //            cv::Mat frame;
    //            cap.read(frame);
    //            assert(frame.type() == CV_8UC1);
    //
    //            cv::imshow("F", frame);
    //            cv::waitKey(0);
    //
    //            NvEncoder::BitstreamView view = nvEncoder.feed(frame);
    //            if (de265_error error = de265_push_data(decoder, view.lockParams.bitstreamBufferPtr, view.lockParams.bitstreamSizeInBytes, clock(), nullptr); error != DE265_OK) {
    //                throw std::runtime_error{"Errored pushing encoder data"};
    //            }
    //        }
    //
    //        const de265_image* image;
    //        do {
    //            image = de265_peek_next_picture(decoder);
    //            if (image) {
    //                int width = de265_get_image_width(image, 0);
    //                int height = de265_get_image_height(image, 0);
    //                int format = de265_get_chroma_format(image);
    //                int bpp = de265_get_bits_per_pixel(image, 0);
    //
    //                if (cv::Size{width, height} != size) throw std::runtime_error{"Unexpected image size"};
    //                if (format != de265_chroma_420) throw std::runtime_error{"Unsupported chroma format"};
    //                if (bpp != 8) throw std::runtime_error{"Unsupported bits per pixel"};
    //
    //                int ystride;
    //                auto* y = de265_get_image_plane(image, 0, &ystride);
    //                cv::Mat Y{height, width, CV_8UC1, (void*) y, static_cast<size_t>(ystride)};
    //                cv::imshow("Y", Y);
    //                cv::waitKey(1);
    //
    //                de265_release_next_picture(decoder);
    //            }
    //        } while (image);
    //    }

    return EXIT_SUCCESS;
}