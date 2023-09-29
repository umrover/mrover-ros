#include <stdio.h>

#include <emscripten/websocket.h>
#include <libde265/de265.h>

EM_BOOL on_open(int event_type, const EmscriptenWebSocketOpenEvent* websocket_event, void* userData) {
    puts("on_open");

    return EM_TRUE;
}
EM_BOOL on_error(int event_type, const EmscriptenWebSocketErrorEvent* websocket_event, void* userData) {
    puts("on_error");

    return EM_TRUE;
}
EM_BOOL on_close(int event_type, const EmscriptenWebSocketCloseEvent* websocket_event, void* userData) {
    puts("on_close");

    return EM_TRUE;
}
EM_BOOL on_message(int event_type, const EmscriptenWebSocketMessageEvent* websocket_event, void* userData) {
    puts("on_message");

    return EM_TRUE;
}

int main() {
    if (!emscripten_websocket_is_supported()) return EXIT_FAILURE;

    EmscriptenWebSocketCreateAttributes create_socket_attributes = {
            .url = "ws://localhost:8080",
            .protocols = "binary",
            .createOnMainThread = EM_TRUE,
    };
    EMSCRIPTEN_WEBSOCKET_T socket = emscripten_websocket_new(&create_socket_attributes);
    if (socket <= 0) return EXIT_FAILURE;

    EMSCRIPTEN_RESULT result = {0};

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