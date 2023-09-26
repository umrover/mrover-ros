// Be careful what you include in this file, it is compiled with nvcc (NVIDIA CUDA compiler)
// For example OpenCV and lie includes cause problems

#include "zed_wrapper.hpp"

namespace mrover {

    // Optimal for the Jetson Xavier NX - this is max threads per block and each block has a max of 2048 threads
    constexpr uint BLOCK_SIZE = 1024;

    /**
     * @brief Runs on the GPU, interleaving the XYZ and BGRA buffers into a single buffer of #Point structs.
     */
    __global__ void fillPointCloudMessageKernel(sl::float4* xyzGpuPtr, sl::uchar4* bgraGpuPtr, Point* pcGpuPtr, size_t size) {
        // This function is invoked once per element at index #i in the point cloud
        size_t i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= size) return;

        pcGpuPtr[i].x = xyzGpuPtr[i].x;
        pcGpuPtr[i].y = xyzGpuPtr[i].y;
        pcGpuPtr[i].z = xyzGpuPtr[i].z;
        pcGpuPtr[i].b = bgraGpuPtr[i].r;
        pcGpuPtr[i].g = bgraGpuPtr[i].g;
        pcGpuPtr[i].r = bgraGpuPtr[i].b;
        pcGpuPtr[i].a = bgraGpuPtr[i].a;
    }

    /**
     * Fills a PointCloud2 message residing on the CPU from two GPU buffers (one for XYZ and one for BGRA).
     *
     * @param xyzGpu    XYZ buffer on the GPU
     * @param bgraGpu   BGRA buffer on the GPU
     * @param pcGpu     Point cloud buffer on the GPU (@see Point)
     * @param msg       Point cloud message with buffer on the CPU
     */
    void fillPointCloudMessageFromGpu(sl::Mat& xyzGpu, sl::Mat& bgraGpu, PointCloudGpu& pcGpu, sensor_msgs::PointCloud2Ptr const& msg) {
        assert(bgraGpu.getWidth() >= xyzGpu.getWidth());
        assert(bgraGpu.getHeight() >= xyzGpu.getHeight());
        assert(bgraGpu.getChannels() == 4);
        assert(xyzGpu.getChannels() == 3);
        assert(msg);

        auto* bgraGpuPtr = bgraGpu.getPtr<sl::uchar4>(sl::MEM::GPU);
        auto* xyzGpuPtr = xyzGpu.getPtr<sl::float4>(sl::MEM::GPU);
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        msg->is_dense = true;
        msg->height = bgraGpu.getHeight();
        msg->width = bgraGpu.getWidth();
        fillPointCloudMessageHeader(msg);
        size_t size = msg->width * msg->height;

        pcGpu.resize(size);
        Point* pcGpuPtr = pcGpu.data().get();
        dim3 threadsPerBlock{BLOCK_SIZE};
        dim3 numBlocks{static_cast<uint>(std::ceil(static_cast<float>(size) / BLOCK_SIZE))};
        fillPointCloudMessageKernel<<<numBlocks, threadsPerBlock>>>(xyzGpuPtr, bgraGpuPtr, pcGpuPtr, size);
        checkCudaError(cudaPeekAtLastError());
        checkCudaError(cudaMemcpy(msg->data.data(), pcGpuPtr, size * sizeof(Point), cudaMemcpyDeviceToHost));
    }

    void checkCudaError(cudaError_t err) {
        if (err == cudaSuccess) return;

        ROS_ERROR_STREAM("CUDA error: " << cudaGetErrorString(err));
        throw std::runtime_error("CUDA error");
    }

} // namespace mrover
