#include <stdexcept>

#include "streaming.hpp"

#include <cuda.h>
#include <nvenv/nvEncodeAPI.h>

void NvCheck(NVENCSTATUS status) {
    if (status != NV_ENC_SUCCESS) {
        throw std::runtime_error("NvEnc failed");
    }
}

Streamer::Streamer() {
    NvCheck(NvEncodeAPICreateInstance(&m_nvenc));
    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params{
            .version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER,
            .deviceType = NV_ENC_DEVICE_TYPE_CUDA,
            .device = nullptr,
            .apiVersion = NVENCAPI_VERSION,
    };
    void* encoder = nullptr;
    NvCheck(m_nvenc.nvEncOpenEncodeSessionEx(&params, &encoder));
}
