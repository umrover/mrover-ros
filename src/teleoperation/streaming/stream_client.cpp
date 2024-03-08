#include <cstdio>
#include <string>

#include <emscripten/val.h>
#include <emscripten/websocket.h>

#include <libde265/de265.h>

de265_decoder_context* decoder{};
;

EMSCRIPTEN_WEBSOCKET_T socket{};

auto create_socket() -> void;

auto on_open(int _event_type, EmscriptenWebSocketOpenEvent const* websocket_event, void* user_data) -> EM_BOOL {
    std::puts("Stream opened");

    return EM_TRUE;
}

auto on_error(int _event_type, EmscriptenWebSocketErrorEvent const* websocket_event, void* user_data) -> EM_BOOL {
    std::puts("Stream errored :(");

    create_socket();

    return EM_TRUE;
}

auto on_close(int _event_type, EmscriptenWebSocketCloseEvent const* websocket_event, void* user_data) -> EM_BOOL {
    std::puts("Stream closed");

    create_socket();

    return EM_TRUE;
}

auto on_message(int _event_type, EmscriptenWebSocketMessageEvent const* websocket_event, void* user_data) -> EM_BOOL {
    if (websocket_event->isText) {
        std::puts("Got text when expected binary");
        return EM_FALSE;
    }

    if (de265_error error = de265_push_data(decoder, websocket_event->data, (int) websocket_event->numBytes, clock(), NULL); error != DE265_OK) {
        std::puts("Errored pushing encoder data");
        return EM_FALSE;
    }

    int more = 0;
    de265_error decode_status;
    while ((decode_status = de265_decode(decoder, &more)) == DE265_OK && more) {
    }

    if (decode_status != DE265_OK && decode_status != DE265_ERROR_WAITING_FOR_INPUT_DATA) {
        std::printf("Errored decoding: %d\n", decode_status);
        return EM_FALSE;
    }

    de265_image const* image;
    while ((image = de265_peek_next_picture(decoder)) != NULL) {
        int width = de265_get_image_width(image, 0);
        int height = de265_get_image_height(image, 0);
        int format = de265_get_chroma_format(image);

        if (format != de265_chroma_420) {
            std::puts("Unsupported chroma format");
            return EM_FALSE;
        }

        int y_stride, u_stride, v_stride;
        std::uint8_t const* y = de265_get_image_plane(image, 0, &y_stride);
        std::uint8_t const* u = de265_get_image_plane(image, 1, &u_stride);
        std::uint8_t const* v = de265_get_image_plane(image, 2, &v_stride);

        // clang-format off
        EM_ASM({
            const width = $0;
            const height = $1;

            const y = HEAPU8.subarray($2, $2 + width * height);
            const u = HEAPU8.subarray($3, $3 + width * height / 4);
            const v = HEAPU8.subarray($4, $4 + width * height / 4);

            const yStride = $5;
            const uStride = $6;
            const vStride = $7;

            const canvas = document.getElementById('canvas');
            const ctx = canvas.getContext('2d');
            if (Module.imageBuffer === undefined) {
                Module.imageBuffer = ctx.createImageData(width, height);
                canvas.width = width;
                canvas.height = height;
            }
            const imageBuffer = Module.imageBuffer;
            const imageData = imageBuffer.data;

            for (let i = 0; i < height; i++) {
                for (let j = 0; j < width; j++) {
                    const yIndex = i * yStride + j;
                    const uIndex = (i / 2 | 0) * uStride + (j / 2 | 0);
                    const vIndex = (i / 2 | 0) * vStride + (j / 2 | 0);

                    const r = y[yIndex] + 1.402 * (v[vIndex] - 128);
                    const g = y[yIndex] - 0.344136 * (u[uIndex] - 128) - 0.714136 * (v[vIndex] - 128);
                    const b = y[yIndex] + 1.772 * (u[uIndex] - 128);

                    const index = (i * width + j) * 4;
                    imageData[index + 0] = r;
                    imageData[index + 1] = g;
                    imageData[index + 2] = b;
                    imageData[index + 3] = 255;
                }
            }

            ctx.putImageData(imageBuffer, 0, 0);

        }, width, height, y, u, v, y_stride, u_stride, v_stride);
        // clang-format on

        de265_release_next_picture(decoder);
    }

    return EM_TRUE;
}

extern "C" {

auto on_before_unload() -> void {
    if (socket <= 0) return;

    emscripten_websocket_close(socket, 1000, "Window is closing");
}
}

auto create_socket() -> void {
    emscripten::val location = emscripten::val::global("location");
    std::string url = "ws://" + location["hostname"].as<std::string>() + ":8080";
    printf("Connecting to %s ...\n", url.c_str());
    EmscriptenWebSocketCreateAttributes create_socket_attributes = {
            .url = url.c_str(),
            .protocols = "binary",
            .createOnMainThread = EM_TRUE,
    };

    socket = emscripten_websocket_new(&create_socket_attributes);
    if (socket <= 0) return;

    EMSCRIPTEN_RESULT result = emscripten_websocket_set_onopen_callback(socket, NULL, on_open);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return;
    result = emscripten_websocket_set_onerror_callback(socket, NULL, on_error);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return;
    result = emscripten_websocket_set_onclose_callback(socket, NULL, on_close);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return;
    result = emscripten_websocket_set_onmessage_callback(socket, NULL, on_message);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return;
}

auto main() -> int {
    if (!emscripten_websocket_is_supported()) {
        fprintf(stderr, "WebSockets are not supported\n");
        return EXIT_FAILURE;
    }

    decoder = de265_new_decoder();
    if (!decoder) {
        fprintf(stderr, "Failed to create libde265 decoder\n");
        return EXIT_FAILURE;
    }

    if (de265_error error = de265_start_worker_threads(decoder, 0); error != DE265_OK) {
        fprintf(stderr, "Failed to start libde265 worker threads: %d\n", error);
        return EXIT_FAILURE;
    }

    create_socket();

    return EXIT_SUCCESS;
}