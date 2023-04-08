#include "zed_wrapper.hpp"

#include <sl/Camera.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace mrover {

    constexpr size_t BLOCK_SIZE = 512;

    __global__ void fillPointCloudMessageKernel(sl::float4* xyzGpuPtr, sl::uchar4* bgraGpuPtr, Point* pcGpuPtr, size_t size) {
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

    void fillPointCloudMessage(sl::Mat& xyzGpu, sl::Mat& bgraGpu, PointCloudGpu& pcGpu, sensor_msgs::PointCloud2Ptr const& msg) {
        assert(bgraGpu.getWidth() >= xyzGpu.getWidth());
        assert(bgraGpu.getHeight() >= xyzGpu.getHeight());
        assert(bgraGpu.getChannels() == 4);
        assert(xyzGpu.getChannels() == 3);
        assert(msg);

        auto bgraGpuPtr = bgraGpu.getPtr<sl::uchar4>(sl::MEM::GPU);
        auto* xyzGpuPtr = xyzGpu.getPtr<sl::float4>(sl::MEM::GPU);
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        msg->is_dense = false;
        msg->height = bgraGpu.getHeight();
        msg->width = bgraGpu.getWidth();
        sensor_msgs::PointCloud2Modifier modifier{*msg};
        modifier.setPointCloud2Fields(
                8,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::PointField::FLOAT32,
                "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                "curvature", 1, sensor_msgs::PointField::FLOAT32);
        size_t size = msg->width * msg->height;

        pcGpu.resize(size);
        Point* pcGpuPtr = pcGpu.data().get();
        fillPointCloudMessageKernel<<<std::ceil(static_cast<float>(size) / BLOCK_SIZE), BLOCK_SIZE>>>(xyzGpuPtr, bgraGpuPtr, pcGpuPtr, size);
        cudaMemcpy(msg->data.data(), pcGpuPtr, size * sizeof(Point), cudaMemcpyDeviceToHost);
    }

} // namespace mrover
