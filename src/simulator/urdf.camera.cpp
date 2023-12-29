#include "simulator.hpp"

namespace mrover {

    auto URDF::makeCameraForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> Camera {
        Camera camera{
                SimulatorNodelet::globalName(name, link->name),
                {320, 240},
                10,
                simulator.mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1),
        };

        glGenFramebuffers(1, &camera.framebufferHandle);
        glBindFramebuffer(GL_FRAMEBUFFER, camera.framebufferHandle);

        GLsizei w = camera.resolution.width, h = camera.resolution.height;

        glGenTextures(1, &camera.colorTextureHandle);
        glBindTexture(GL_TEXTURE_2D, camera.colorTextureHandle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        // Following are needed for ImGui to successfully render
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        glGenTextures(1, &camera.depthTextureHandle);
        glBindTexture(GL_TEXTURE_2D, camera.depthTextureHandle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, camera.depthTextureHandle, 0);

        // glGenRenderbuffers(1, &camera.depthTextureHandle);
        // glBindRenderbuffer(GL_RENDERBUFFER, camera.depthTextureHandle);
        // glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
        //
        // // Attach the depth texture to the framebuffer
        // glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, camera.depthTextureHandle);

        // Attach the color texture to the framebuffer
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, camera.colorTextureHandle, 0);

        // std::array<GLenum, 2> attachments{GL_COLOR_ATTACHMENT0, GL_DEPTH_ATTACHMENT};
        // glDrawBuffers(attachments.size(), attachments.data());

        glGenBuffers(1, &camera.pointCloudArrayHandle);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, camera.pointCloudArrayHandle);
        glBufferData(GL_SHADER_STORAGE_BUFFER, w * h * sizeof(Point), nullptr, GL_DYNAMIC_COPY);

        if (GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER); status != GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error{std::format("Framebuffer incomplete: {:#x}", status)};

        return camera;
    }

} // namespace mrover
