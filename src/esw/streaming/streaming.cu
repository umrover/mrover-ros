#include <stdexcept>

#include "streaming.hpp"

#include <cuda.h>
#include <nvEncodeAPI.h>

void NvCheck(NVENCSTATUS status) {
    if (status != NV_ENC_SUCCESS) {
        throw std::runtime_error("NvEnc failed");
    }
}

void CuCheck(CUresult status) {
    if (status != CUDA_SUCCESS) {
        throw std::runtime_error("CUDA failed");
    }
}

Streamer::Streamer() {
    NvCheck(NvEncodeAPICreateInstance(&m_nvenc));
    CuCheck(cuInit(0));
    CUcontext context;
    CuCheck(cuCtxCreate(&context, 0, 0));
    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params{
            .version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER,
            .deviceType = NV_ENC_DEVICE_TYPE_CUDA,
            .device = &context,
            .apiVersion = NVENCAPI_VERSION,
    };
    void* encoder = nullptr;
    NvCheck(m_nvenc.nvEncOpenEncodeSessionEx(&params, &encoder));
}
