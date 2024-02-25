#include "gst_nv_h265_enc.hpp"

#include <cuda.h>

#include <cassert>

CUcontext CUDA_CONTEXT = nullptr;

void init_cuda() {
    auto result = cuInit(0);
    assert(result == CUDA_SUCCESS);
}

