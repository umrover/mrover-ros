#pragma once

#include <chrono>

#include <nvEncodeAPI.h>
#include <opencv2/core/mat.hpp>

class NvVideoCodecEncoder {
public:
    struct BitstreamView {
        NV_ENCODE_API_FUNCTION_LIST* nvenc = nullptr;
        void* encoder = nullptr;
        NV_ENC_OUTPUT_PTR output = nullptr;
        NV_ENC_LOCK_BITSTREAM lockParams{};

        BitstreamView(NV_ENCODE_API_FUNCTION_LIST* nvenc, void* encoder, NV_ENC_OUTPUT_PTR output);

        ~BitstreamView();

        BitstreamView(BitstreamView const&) = delete;
        auto operator=(BitstreamView const&) -> BitstreamView& = delete;

        BitstreamView(BitstreamView&& other) noexcept;
        auto operator=(BitstreamView&& other) noexcept -> BitstreamView&;
    };

private:
    cv::Size m_size;
    void* m_encoder = nullptr;
    NV_ENC_INPUT_PTR m_input = nullptr;
    NV_ENC_OUTPUT_PTR m_output = nullptr;
    std::uint32_t m_frame_index = 0;
    std::chrono::high_resolution_clock m_clock;

public:
    explicit NvVideoCodecEncoder(cv::Size const& size);

    [[nodiscard]] auto feed(cv::InputArray frameBgra) -> BitstreamView;

    ~NvVideoCodecEncoder();
};
