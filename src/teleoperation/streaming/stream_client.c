#include <stdio.h>

#include <emscripten/websocket.h>
#include <libde265/de265.h>

de265_decoder_context* decoder = NULL;

EM_BOOL on_open(int event_type, const EmscriptenWebSocketOpenEvent* websocket_event, void* user_data) {
    puts("Stream opened");

    return EM_TRUE;
}

EM_BOOL on_error(int event_type, const EmscriptenWebSocketErrorEvent* websocket_event, void* user_data) {
    printf("Stream errored :( %d\n", event_type);

    return EM_TRUE;
}

EM_BOOL on_close(int event_type, const EmscriptenWebSocketCloseEvent* websocket_event, void* user_data) {
    puts("Stream closed");

    return EM_TRUE;
}

EM_BOOL on_message(int event_type, const EmscriptenWebSocketMessageEvent* websocket_event, void* user_data) {
    de265_error error = de265_push_data(decoder, websocket_event->data, websocket_event->numBytes, clock(), NULL);
    if (error != DE265_OK) {
        puts("Errored pushing encoder data");
        return EM_FALSE;
    }

    int more = 0;
    de265_error decode_status;
    while ((decode_status = de265_decode(decoder, &more)) == DE265_OK && more) {

        if (decode_status != DE265_OK && decode_status != DE265_ERROR_WAITING_FOR_INPUT_DATA) {
            printf("Errored decoding: %d\n", decode_status);
            return EM_FALSE;
        }

        const struct de265_image* image = de265_get_next_picture(decoder);

        if (image != NULL) {
            int width = de265_get_image_width(image, 0);
            int height = de265_get_image_height(image, 0);
            int format = de265_get_chroma_format(image);

            printf("Got image: %dx%d %d\n", width, height, format);

            if (format != de265_chroma_420) {
                puts("Unsupported chroma format");
                return EM_FALSE;
            }
        }
    }

    return EM_TRUE;
}

int main() {
    if (!emscripten_websocket_is_supported()) return EXIT_FAILURE;

    decoder = de265_new_decoder();
    if (!decoder) return EXIT_FAILURE;

    de265_error error = de265_start_worker_threads(decoder, 0);
    if (error != DE265_OK) return EXIT_FAILURE;

    EmscriptenWebSocketCreateAttributes create_socket_attributes = {
            .url = "ws://127.0.0.1:8080",
            .protocols = "binary",
            .createOnMainThread = EM_TRUE,
    };
    EMSCRIPTEN_WEBSOCKET_T socket = emscripten_websocket_new(&create_socket_attributes);
    if (socket <= 0) return EXIT_FAILURE;

    EMSCRIPTEN_RESULT result;
    result = emscripten_websocket_set_onopen_callback(socket, NULL, on_open);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return EXIT_FAILURE;
    result = emscripten_websocket_set_onerror_callback(socket, NULL, on_error);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return EXIT_FAILURE;
    result = emscripten_websocket_set_onclose_callback(socket, NULL, on_close);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return EXIT_FAILURE;
    result = emscripten_websocket_set_onmessage_callback(socket, NULL, on_message);
    if (result != EMSCRIPTEN_RESULT_SUCCESS) return EXIT_FAILURE;

    return EXIT_SUCCESS;
}