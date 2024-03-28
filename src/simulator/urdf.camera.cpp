#include "simulator.hpp"

namespace mrover {

    auto URDF::makeCameraForLink(SimulatorNodelet& simulator, btMultibodyLink const* link) -> Camera {
        Camera camera;
        camera.link = link;
        camera.resolution = {640, 480};
        camera.updateTask = PeriodicTask{20};
        // TODO(quintin): Why do I have to cast this
        wgpu::TextureUsage usage = static_cast<wgpu::TextureUsage::W>(wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::TextureBinding);
        wgpu::TextureUsage colorUsage = static_cast<wgpu::TextureUsage::W>(usage | wgpu::TextureUsage::CopySrc);
        std::tie(camera.colorTexture, camera.colorTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), COLOR_FORMAT, colorUsage, wgpu::TextureAspect::All);
        std::tie(camera.normalTexture, camera.normalTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), NORMAL_FORMAT, usage, wgpu::TextureAspect::All);
        std::tie(camera.depthTexture, camera.depthTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), DEPTH_FORMAT, usage, wgpu::TextureAspect::DepthOnly);
        return camera;
    }

} // namespace mrover
