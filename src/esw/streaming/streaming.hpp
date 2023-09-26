#pragma once

#include <nvenv/nvEncodeAPI.h>

struct Streamer {

    NV_ENCODE_API_FUNCTION_LIST m_nvenc{NV_ENCODE_API_FUNCTION_LIST_VER};

    Streamer();
};