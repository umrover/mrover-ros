#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "streaming.hpp"

#include <cuda.h>
#include <nvEncodeAPI.h>

void NvCheck(NVENCSTATUS status) {
    if (status != NV_ENC_SUCCESS) {
        throw std::runtime_error("NvEnc failed");
    }
}

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

namespace std {
    template<>
    struct equal_to<GUID> {
        bool operator()(GUID const& g1, GUID const& g2) const {
            return g1.Data1 == g2.Data1 && g1.Data2 == g2.Data2 && g1.Data3 == g2.Data3 &&
                   g1.Data4[0] == g2.Data4[0] && g1.Data4[1] == g2.Data4[1] && g1.Data4[2] == g2.Data4[2] &&
                   g1.Data4[3] == g2.Data4[3] && g1.Data4[4] == g2.Data4[4] && g1.Data4[5] == g2.Data4[5] &&
                   g1.Data4[6] == g2.Data4[6] && g1.Data4[7] == g2.Data4[7];
        }
    };

    template<>
    struct hash<GUID> {
        std::size_t operator()(GUID const& g) const {
            std::size_t seed = 0;
            seed ^= std::hash<std::uint32_t>{}(g.Data1);
            seed ^= std::hash<std::uint16_t>{}(g.Data2);
            seed ^= std::hash<std::uint16_t>{}(g.Data3);
            for (std::size_t i = 0; i < 8; ++i) {
                seed ^= std::hash<std::uint8_t>{}(g.Data4[i]);
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

Streamer::Streamer(std::uint32_t width, std::uint32_t height) {
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
    void* encoder = nullptr;
    NvCheck(m_nvenc.nvEncOpenEncodeSessionEx(&params, &encoder));
    if (!encoder) {
        throw std::runtime_error("No encoder");
    }

    std::uint32_t guidCount;
    NvCheck(m_nvenc.nvEncGetEncodeGUIDCount(encoder, &guidCount));
    if (guidCount == 0) {
        throw std::runtime_error("No GUIDs");
    }

    std::vector<GUID> guids(guidCount);
    NvCheck(m_nvenc.nvEncGetEncodeGUIDs(encoder, guids.data(), guidCount, &guidCount));
    std::cout << "Supported encoders:" << std::endl;
    for (GUID const& guid: guids) {
        std::cout << "\t" << GUID_TO_NAME[guid] << std::endl;
    }

    GUID desiredEncodeGuid = NV_ENC_CODEC_HEVC_GUID;
    GUID desiredPresetGuid = NV_ENC_PRESET_P4_GUID;

    if (std::none_of(guids.begin(), guids.end(), [&](const GUID& guid) {
            return std::equal_to<GUID>{}(guid, desiredEncodeGuid);
        })) {
        throw std::runtime_error("No HEVC GUID");
    }

    std::uint32_t presetCount;
    NvCheck(m_nvenc.nvEncGetEncodePresetCount(encoder, desiredEncodeGuid, &presetCount));
    std::vector<GUID> presetGuids(presetCount);
    NvCheck(m_nvenc.nvEncGetEncodePresetGUIDs(encoder, desiredEncodeGuid, presetGuids.data(), presetCount, &presetCount));
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
    NvCheck(m_nvenc.nvEncGetEncodePresetConfigEx(encoder, desiredEncodeGuid, desiredPresetGuid, tuningInfo, &presetConfig));

    NV_ENC_INITIALIZE_PARAMS encInitParams{
            .version = NV_ENC_INITIALIZE_PARAMS_VER,
            .encodeGUID = desiredEncodeGuid,
            .presetGUID = desiredPresetGuid,
            .encodeWidth = width,
            .encodeHeight = height,
            .darWidth = width,
            .darHeight = height,
            .frameRateNum = 30,
            .frameRateDen = 1,
            .tuningInfo = tuningInfo,
            .encodeConfig = &presetConfig.presetCfg,
    };
    NvCheck(m_nvenc.nvEncInitializeEncoder(encoder, &encInitParams));
}
