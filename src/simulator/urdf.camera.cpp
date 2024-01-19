#include "simulator.hpp"

namespace mrover {

    auto URDF::makeCameraForLink(SimulatorNodelet& simulator, btMultibodyLink const* link) -> Camera {
        Camera camera{
                link,
                {640 / 2, 480 / 2},
                PeriodicTask{30},
                simulator.mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1),
        };
        // TODO(quintin): Why do I have to cast this
        wgpu::TextureUsage usage = static_cast<wgpu::TextureUsage::W>(wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::TextureBinding);
        std::tie(camera.colorTexture, camera.colorTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), COLOR_FORMAT, usage, wgpu::TextureAspect::All);
        std::tie(camera.normalTexture, camera.normalTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), NORMAL_FORMAT, usage, wgpu::TextureAspect::All);
        std::tie(camera.depthTexture, camera.depthTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), DEPTH_FORMAT, usage, wgpu::TextureAspect::DepthOnly);
        return camera;
    }

} // namespace mrover
