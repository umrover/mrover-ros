#include <encoding.hpp>
#include <streaming.hpp>

#include <format>
#include <iostream>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <libde265/de265.h>

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

    //    cv::VideoCapture cap{
    //            "v4l2src ! videoconvert ! video/x-raw,width=320,height=240,format=I420,framerate=30/1 ! appsink",
    //            cv::CAP_GSTREAMER};

    cv::VideoCapture cap{"videotestsrc ! video/x-raw,width=320,height=240,format=I420,framerate=30/1 ! appsink", cv::CAP_GSTREAMER};

    if (!cap.isOpened()) throw std::runtime_error{"Failed to open capture"};

    static de265_decoder_context* decoder = de265_new_decoder();
    if (!decoder) return EXIT_FAILURE;
    if (de265_error error = de265_start_worker_threads(decoder, 0); error != DE265_OK) throw std::runtime_error{"Errored starting worker threads"};

    int more = 1;
    while (more) {
        de265_error status = de265_decode(decoder, &more);

        if (status == DE265_ERROR_WAITING_FOR_INPUT_DATA) {
            cv::Mat frame;
            cap.read(frame);
            assert(frame.type() == CV_8UC1);

            cv::imshow("F", frame);
            cv::waitKey(1);

            Encoder::BitstreamView view = nvencEncoder.feed(frame);
            if (de265_error error = de265_push_data(decoder, view.lockParams.bitstreamBufferPtr, view.lockParams.bitstreamSizeInBytes, clock(), nullptr); error != DE265_OK) {
                throw std::runtime_error{"Errored pushing encoder data"};
            }
        }

        const de265_image* image;
        do {
            image = de265_peek_next_picture(decoder);
            if (image) {
                int width = de265_get_image_width(image, 0);
                int height = de265_get_image_height(image, 0);
                int format = de265_get_chroma_format(image);
                int bpp = de265_get_bits_per_pixel(image, 0);

                if (format != de265_chroma_420) throw std::runtime_error{"Unsupported chroma format"};

                int ystride;
                auto* y = de265_get_image_plane(image, 0, &ystride);
                cv::Mat Y{height, width, CV_8UC1, (void*) y, static_cast<size_t>(ystride)};
                cv::imshow("Y", Y);
                cv::waitKey(1);

                de265_release_next_picture(decoder);
            }
        } while (image);
    }

    return EXIT_SUCCESS;
}