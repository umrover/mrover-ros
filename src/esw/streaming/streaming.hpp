#pragma once

#include "pch.hpp"

using Size32u = cv::Size_<std::uint32_t>;

struct Streamer {

    Size32u m_size;
    NV_ENCODE_API_FUNCTION_LIST m_nvenc{NV_ENCODE_API_FUNCTION_LIST_VER};
    void* m_encoder = nullptr;
    NV_ENC_INPUT_PTR m_input = nullptr;
    NV_ENC_OUTPUT_PTR m_output = nullptr;
    uint32_t m_frame_index = 0;
    std::chrono::high_resolution_clock m_clock;

    Streamer(Size32u const& size);

    void feed(cv::InputArray frame);

    ~Streamer();
};