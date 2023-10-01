#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <iostream>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <libde265/de265.h>
#include <libde265/en265.h>

cv::Size size{320, 240};

int main() {
    //    Streamer streamer{"127.0.0.1", 8080};

    Encoder nvencEncoder{size};

    //    cv::VideoCapture cap{
    //            0,
    //            cv::CAP_V4L2,
    //            {cv::CAP_PROP_FRAME_WIDTH, size.width,
    //             cv::CAP_PROP_FRAME_HEIGHT, size.height,
    //             cv::CAP_PROP_FPS, 30,
    //             cv::CAP_PROP_CONVERT_RGB, false,
    //             //             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2')},
    //             cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('I', '4', '2', '0')},
    //    };

    cv::VideoCapture cap{
            "v4l2src ! videoconvert ! video/x-raw,width=320,height=240,format=I420,framerate=30/1 ! appsink",
            cv::CAP_GSTREAMER};

    //    cv::VideoCapture cap{"videotestsrc ! video/x-raw,width=640,height=480,format=I420,framerate=30/1 ! appsink", cv::CAP_GSTREAMER};

    if (!cap.isOpened()) {
        throw std::runtime_error{"Failed to open capture"};
    }

    static de265_decoder_context* decoder = de265_new_decoder();
    if (!decoder) return EXIT_FAILURE;
    if (de265_error error = de265_start_worker_threads(decoder, 0); error != DE265_OK) throw std::runtime_error{"Errored starting worker threads"};

    //    static en265_encoder_context* encoder = en265_new_encoder();
    //    if (!encoder) throw std::runtime_error{"Errored creating encoder"};
    //    if (de265_error error = en265_start_encoder(encoder, 12); error != DE265_OK) throw std::runtime_error{"Errored starting encoder"};
    //
    //    static de265_image* encode_image = en265_allocate_image(encoder, size.width, size.height, de265_chroma_420, 0, nullptr);
    //    if (!encode_image) throw std::runtime_error{"Errored allocating image"};

    cv::Mat frame;
    while (cap.read(frame)) {
        assert(frame.type() == CV_8UC1);

        //        auto* y = de265_get_image_plane(encode_image, 0, nullptr);
        //        auto* u = de265_get_image_plane(encode_image, 1, nullptr);
        //        auto* v = de265_get_image_plane(encode_image, 2, nullptr);
        //        std::memcpy((void*) y, frame.data, size.area());0
        //        std::memcpy((void*) u, frame.data + size.area(), size.area() / 4);
        //        std::memcpy((void*) v, frame.data + size.area() + size.area() / 4, size.area() / 4);

        //        if (de265_error error = en265_push_image(encoder, encode_image); error != DE265_OK) throw std::runtime_error{"Errored pushing encoder image"};
        //        if (de265_error error = en265_encode(encoder); error != DE265_OK) throw std::runtime_error{"Errored encoding"};

        //        en265_packet* packet;
        //        while ((packet = en265_get_packet(encoder, 0))) {
        //            std::cout << std::format("Got packet of size {}\n", packet->length);
        //            de265_error error = de265_push_data(decoder, packet->data, packet->length, 0, nullptr);
        //            if (error != DE265_OK) {
        //                throw std::runtime_error{"Errored pushing encoder data"};
        //            }
        //        }

        Encoder::BitstreamView view = nvencEncoder.feed(frame);
        std::span<std::byte> span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};

        de265_error error = de265_push_data(decoder, span.data(), span.size(), clock(), NULL);
        if (error != DE265_OK) {
            throw std::runtime_error{"Errored pushing encoder data"};
        }

        //        streamer.feed(span);

        int more = 0;
        de265_error decode_status;
        while ((decode_status = de265_decode(decoder, &more)) == DE265_OK && more) {
        }

        if (decode_status != DE265_OK && decode_status != DE265_ERROR_WAITING_FOR_INPUT_DATA) {
            throw std::runtime_error{"Error decoding"};
        }

        const struct de265_image* image;
        while ((image = de265_peek_next_picture(decoder))) {
            int width = de265_get_image_width(image, 0);
            int height = de265_get_image_height(image, 0);
            int format = de265_get_chroma_format(image);

            if (format != de265_chroma_420) {
                throw std::runtime_error{"Unsupported chroma format"};
            }

            int ystride, ustride, vstride;
            auto* y = de265_get_image_plane(image, 0, &ystride);
            cv::Mat Y{height, width, CV_8UC1, (void*) y};
            //            cv::Mat Y = cv::Mat::zeros(height, width, CV_8UC1);
            //            for (int i = 0; i < width; ++i) {
            //                cv::Mat row = cv::Mat{height, 1, CV_8UC1, (void*) (y + i * ystride)};
            //                row.copyTo(Y.col(i));
            //            }
            cv::imshow("Y", Y);
            cv::imshow("F", frame);
            cv::waitKey(1);

            de265_release_next_picture(decoder);
        }
    }

    return EXIT_SUCCESS;
}