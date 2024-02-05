#include "hardware_h265_encoder.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <cuda.h>
#include <nvEncodeAPI.h>

void cuCheck(CUresult status) {
    if (status != CUDA_SUCCESS) {
        throw std::runtime_error("CUDA failed");
    }
}

void cudaCheck(cudaError_t status) {
    if (status != cudaSuccess) {
        throw std::runtime_error("CUDA failed");
    }
}

void NvCheck(NVENCSTATUS status) {
    if (status != NV_ENC_SUCCESS) {
        throw std::runtime_error("NvEnc failed");
    }
}

namespace std {
    template<>
    struct equal_to<GUID> {
        auto operator()(GUID const& g1, GUID const& g2) const -> bool {
            return g1.Data1 == g2.Data1 && g1.Data2 == g2.Data2 && g1.Data3 == g2.Data3 &&
                   g1.Data4[0] == g2.Data4[0] && g1.Data4[1] == g2.Data4[1] && g1.Data4[2] == g2.Data4[2] &&
                   g1.Data4[3] == g2.Data4[3] && g1.Data4[4] == g2.Data4[4] && g1.Data4[5] == g2.Data4[5] &&
                   g1.Data4[6] == g2.Data4[6] && g1.Data4[7] == g2.Data4[7];
        }
    };

    template<>
    struct hash<GUID> {
        auto operator()(GUID const& g) const noexcept -> std::size_t {
            std::size_t seed = 0;
            seed ^= std::hash<std::uint32_t>{}(g.Data1);
            seed ^= std::hash<std::uint16_t>{}(g.Data2);
            seed ^= std::hash<std::uint16_t>{}(g.Data3);
            for (unsigned char i: g.Data4) {
                seed ^= std::hash<std::uint8_t>{}(i);
            }
            return seed;
        }
    };
} // namespace std

std::unordered_map<GUID, std::string> GUID_TO_NAME{
        {NV_ENC_CODEC_HEVC_GUID, "HEVC"},
        {NV_ENC_CODEC_H264_GUID, "H264"},
        {NV_ENC_CODEC_AV1_GUID, "AV1"},
};

Encoder::Encoder(cv::Size const& size) : m_size{size} {
    cudaCheck(cudaSetDevice(0));
    CUcontext context;
    cuCheck(cuCtxGetCurrent(&context));

    NvCheck(NvEncodeAPICreateInstance(&m_nvenc));
    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params{
            .version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER,
            .deviceType = NV_ENC_DEVICE_TYPE_CUDA,
            .device = context,
            .apiVersion = NVENCAPI_VERSION,
    };
    NvCheck(m_nvenc.nvEncOpenEncodeSessionEx(&params, &m_encoder));
    if (!m_encoder) {
        throw std::runtime_error("No encoder");
    }

    std::uint32_t guidCount;
    NvCheck(m_nvenc.nvEncGetEncodeGUIDCount(m_encoder, &guidCount));
    if (guidCount == 0) {
        throw std::runtime_error("No GUIDs");
    }

    std::vector<GUID> guids(guidCount);
    NvCheck(m_nvenc.nvEncGetEncodeGUIDs(m_encoder, guids.data(), guidCount, &guidCount));
    std::cout << "Supported encoders:" << std::endl;
    for (GUID const& guid: guids) {
        std::cout << "\t" << GUID_TO_NAME[guid] << std::endl;
    }

    GUID desiredEncodeGuid = NV_ENC_CODEC_HEVC_GUID;
    GUID desiredPresetGuid = NV_ENC_PRESET_P3_GUID;

    if (std::none_of(guids.begin(), guids.end(), [&](const GUID& guid) {
            return std::equal_to<GUID>{}(guid, desiredEncodeGuid);
        })) {
        throw std::runtime_error("No HEVC GUID");
    }

    std::uint32_t presetCount;
    NvCheck(m_nvenc.nvEncGetEncodePresetCount(m_encoder, desiredEncodeGuid, &presetCount));
    std::vector<GUID> presetGuids(presetCount);
    NvCheck(m_nvenc.nvEncGetEncodePresetGUIDs(m_encoder, desiredEncodeGuid, presetGuids.data(), presetCount, &presetCount));
    if (std::none_of(presetGuids.begin(), presetGuids.end(), [&](const GUID& guid) {
            return std::equal_to<GUID>{}(guid, desiredPresetGuid);
        })) {
        throw std::runtime_error("No P4 preset");
    }

    NV_ENC_TUNING_INFO tuningInfo = NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY;
    NV_ENC_PRESET_CONFIG presetConfig{
            .version = NV_ENC_PRESET_CONFIG_VER,
            .presetCfg = {
                    .version = NV_ENC_CONFIG_VER,
            },
    };
    NvCheck(m_nvenc.nvEncGetEncodePresetConfigEx(m_encoder, desiredEncodeGuid, desiredPresetGuid, tuningInfo, &presetConfig));

    NV_ENC_INITIALIZE_PARAMS initEncParams{
            .version = NV_ENC_INITIALIZE_PARAMS_VER,
            .encodeGUID = desiredEncodeGuid,
            .presetGUID = desiredPresetGuid,
            .encodeWidth = static_cast<std::uint32_t>(m_size.width),
            .encodeHeight = static_cast<std::uint32_t>(m_size.height),
            .frameRateNum = 30,
            .frameRateDen = 1,
            .enablePTD = true,
            .encodeConfig = &presetConfig.presetCfg,
            .tuningInfo = tuningInfo,
    };
    NvCheck(m_nvenc.nvEncInitializeEncoder(m_encoder, &initEncParams));

    NV_ENC_CREATE_INPUT_BUFFER createInputBufferParams{
            .version = NV_ENC_CREATE_INPUT_BUFFER_VER,
            .width = static_cast<std::uint32_t>(m_size.width),
            .height = static_cast<std::uint32_t>(m_size.height),
            .bufferFmt = NV_ENC_BUFFER_FORMAT_ARGB,
    };
    NvCheck(m_nvenc.nvEncCreateInputBuffer(m_encoder, &createInputBufferParams));
    m_input = createInputBufferParams.inputBuffer;

    NV_ENC_CREATE_BITSTREAM_BUFFER createBitstreamBufferParams{
            .version = NV_ENC_CREATE_BITSTREAM_BUFFER_VER,
    };
    NvCheck(m_nvenc.nvEncCreateBitstreamBuffer(m_encoder, &createBitstreamBufferParams));
    m_output = createBitstreamBufferParams.bitstreamBuffer;
}

auto Encoder::feed(cv::InputArray frameBgra) -> BitstreamView {
    if (!frameBgra.isContinuous()) throw std::runtime_error("Frame is not continuous");
    // if (frameI420.type() != CV_8UC1) throw std::runtime_error("Not single channel, note that YUV420 is expected");
    // if (frameI420.size() != cv::Size{m_size.width, m_size.height + m_size.height / 2}) throw std::runtime_error("Wrong size, note that YUV420 is expected");

    NV_ENC_LOCK_INPUT_BUFFER lockInputBufferParams{
            .version = NV_ENC_LOCK_INPUT_BUFFER_VER,
            .inputBuffer = m_input,
    };
    NvCheck(m_nvenc.nvEncLockInputBuffer(m_encoder, &lockInputBufferParams));
    for (int r = 0; r < frameBgra.rows(); ++r) {
        auto* row = static_cast<std::byte*>(lockInputBufferParams.bufferDataPtr) + r * lockInputBufferParams.pitch;
        std::memcpy(row, frameBgra.getMat().ptr(r), frameBgra.cols() * 4);
    }
    NvCheck(m_nvenc.nvEncUnlockInputBuffer(m_encoder, m_input));

    NV_ENC_PIC_PARAMS picParams{
            .version = NV_ENC_PIC_PARAMS_VER,
            .inputWidth = static_cast<std::uint32_t>(m_size.width),
            .inputHeight = static_cast<std::uint32_t>(m_size.height),
            .frameIdx = m_frame_index++,
            .inputTimeStamp = static_cast<std::uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
            .inputBuffer = m_input,
            .outputBitstream = m_output,
            .completionEvent = nullptr,
            .bufferFmt = NV_ENC_BUFFER_FORMAT_IYUV,
            .pictureStruct = NV_ENC_PIC_STRUCT_FRAME,
    };

    NvCheck(m_nvenc.nvEncEncodePicture(m_encoder, &picParams));

    return {&m_nvenc, m_encoder, m_output};
}

Encoder::BitstreamView::BitstreamView(NV_ENCODE_API_FUNCTION_LIST* nvenc, void* encoder, NV_ENC_OUTPUT_PTR output)
    : nvenc{nvenc}, encoder{encoder}, output{output}, lockParams{.version = NV_ENC_LOCK_BITSTREAM_VER, .outputBitstream = output} {
    NvCheck(nvenc->nvEncLockBitstream(encoder, &lockParams));
}

Encoder::BitstreamView::~BitstreamView() {
    if (nvenc && encoder && output) {
        NvCheck(nvenc->nvEncUnlockBitstream(encoder, output));
    }
}

Encoder::BitstreamView::BitstreamView(BitstreamView&& other) noexcept {
    *this = std::move(other);
}

auto Encoder::BitstreamView::operator=(BitstreamView&& other) noexcept -> BitstreamView& {
    std::swap(nvenc, other.nvenc);
    std::swap(encoder, other.encoder);
    std::swap(output, other.output);
    std::swap(lockParams, other.lockParams);
    return *this;
}

Encoder::~Encoder() {
    NvCheck(m_nvenc.nvEncDestroyInputBuffer(m_encoder, m_input));
    NvCheck(m_nvenc.nvEncDestroyBitstreamBuffer(m_encoder, m_output));
    NvCheck(m_nvenc.nvEncDestroyEncoder(m_encoder));
}
