#pragma once

#include "pch.hpp"

class SimulatorNodelet;

namespace mrover {

    /**
     * \brief       Buffer that exists on the CPU and GPU
     * \tparam T    Element type
     *
     * \note There should be a concept requiring std::is_trivially_copyable_v<T> but for some reason Eigen types are not!
     */
    template<typename T>
    struct SharedBuffer {
        using value_type = T;

        std::vector<T> data;

        wgpu::Buffer buffer = nullptr;

        auto ensureUploaded(wgpu::Device& device, wgpu::Queue& queue, wgpu::BufferUsage const& usage) -> bool {
            assert(device);
            assert(queue);

            if (data.empty()) return false;
            if (buffer != nullptr) return false;

            wgpu::BufferDescriptor descriptor;
            descriptor.usage = usage | wgpu::BufferUsage::CopyDst;
            descriptor.size = static_cast<std::uint32_t>(data.size() * sizeof(T));
            buffer = device.createBuffer(descriptor);

            queue.writeBuffer(buffer, 0, data.data(), descriptor.size);

            return true;
        }

        [[nodiscard]] auto sizeBytes() const -> std::size_t {
            return data.size() * sizeof(T);
        }
    };

    // TODO(quintin): clean up shared behavior between this and other types in this file
    template<typename T>
    struct Uniform {
        T data{};

        wgpu::Device device = nullptr;
        wgpu::Queue queue = nullptr;

        wgpu::Buffer buffer = nullptr;

        auto init(wgpu::Device& device, wgpu::Queue const& queue) {
            this->device = device;
            this->queue = queue;

            wgpu::BufferDescriptor descriptor;
            descriptor.usage = wgpu::BufferUsage::Uniform | wgpu::BufferUsage::CopyDst;
            descriptor.size = sizeof(T);
            buffer = device.createBuffer(descriptor);
        }

        auto update(T const& value) -> void {
            assert(queue);
            assert(buffer);

            queue.writeBuffer(buffer, 0, &value, sizeof(T));
        }
    };

    struct SharedTexture {
        cv::Mat data;

        auto prepare() -> bool {
            if (data.empty()) return false;
            // if (handle != GL_INVALID_HANDLE) return false;

            // // See: https://learnopengl.com/Getting-started/Textures

            // // OpenCV's (0, 0) is in the top-left, but OpenGL's is bottom-left
            // flip(data, data, 0);

            // glGenTextures(1, &handle);
            // assert(handle != GL_INVALID_HANDLE);
            // glBindTexture(GL_TEXTURE_2D, handle);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // // Upload to the GPU
            // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data.cols, data.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, data.data);
            // // Mipmaps make far away textures look better
            // glGenerateMipmap(GL_TEXTURE_2D);

            return true;
        }
    };

} // namespace mrover
