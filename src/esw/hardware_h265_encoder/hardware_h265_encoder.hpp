#pragma once

#include <chrono>

#include <nvEncodeAPI.h>
#include <opencv2/core/mat.hpp>

class Encoder {
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
    NV_ENCODE_API_FUNCTION_LIST m_nvenc{.version = NV_ENCODE_API_FUNCTION_LIST_VER};
    void* m_encoder = nullptr;
    NV_ENC_INPUT_PTR m_input = nullptr;
    NV_ENC_OUTPUT_PTR m_output = nullptr;
    std::uint32_t m_frame_index = 0;
    std::chrono::high_resolution_clock m_clock;

public:
    explicit Encoder(cv::Size const& size);

    [[nodiscard]] auto feed(cv::InputArray frameBgra) -> BitstreamView;

    ~Encoder();
};
