#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <iostream>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <libde265/de265.h>

cv::Size size{640, 480};

int main() {
    Streamer streamer{"127.0.0.1", 8080};

    Encoder encoder{size};

    //    cv::VideoCapture cap{
    //            0,
    //            cv::CAP_V4L2,
    //            {cv::CAP_PROP_FRAME_WIDTH, size.width,
    //             cv::CAP_PROP_FRAME_HEIGHT, size.height,
    //             cv::CAP_PROP_FPS, 60,
    //             cv::CAP_PROP_CONVERT_RGB, false,
    //             //             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2')},
    //             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'U', 'V')},
    //    };
    //    if (!cap.isOpened()) {
    //        throw std::runtime_error{"Failed to open camera"};
    //    }

    cv::VideoCapture cap{"videotestsrc ! video/x-raw,width=640,height=480,format=I420,framerate=30/1 ! appsink", cv::CAP_GSTREAMER};

    //    static de265_decoder_context* decoder = NULL;
    //    decoder = de265_new_decoder();
    //    if (!decoder) return EXIT_FAILURE;
    //    de265_error error = de265_start_worker_threads(decoder, 0);
    //    if (error != DE265_OK) return EXIT_FAILURE;

    cv::Mat frame;
    while (cap.read(frame)) {
        Encoder::BitstreamView view = encoder.feed(frame);
        std::span<std::byte> span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        std::cout << std::format("Sending {} bytes\n", span.size());
        streamer.feed(span);

        //        de265_error error = de265_push_data(decoder, span.data(), span.size(), clock(), NULL);
        //        if (error != DE265_OK) {
        //            throw std::runtime_error{"Errored pushing encoder data"};
        //        }
        //
        //        int more = 0;
        //        de265_error decode_status;
        //        while ((decode_status = de265_decode(decoder, &more)) == DE265_OK && more) {
        //        }
        //
        //        if (decode_status != DE265_OK && decode_status != DE265_ERROR_WAITING_FOR_INPUT_DATA) {
        //            throw std::runtime_error{"Error decoding"};
        //        }
        //
        //        const struct de265_image* image;
        //        while ((image = de265_peek_next_picture(decoder)) != NULL) {
        //            int width = de265_get_image_width(image, 0);
        //            int height = de265_get_image_height(image, 0);
        //            int format = de265_get_chroma_format(image);
        //
        //            printf("Got image: %dx%d %d\n", width, height, format);
        //
        //            if (format != de265_chroma_420) {
        //                throw std::runtime_error{"Unsupported chroma format"};
        //            }
        //
        //            de265_release_next_picture(decoder);
        //        }
    }

    return EXIT_SUCCESS;
}