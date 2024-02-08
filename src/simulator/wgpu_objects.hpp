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

        SharedBuffer() = default;

        auto enqueueWriteIfUnitialized(wgpu::Device& device, wgpu::Queue& queue, wgpu::BufferUsage const& usage) -> bool {
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

        ~SharedBuffer() {
            if (buffer) {
                buffer.destroy();
                buffer.release();
            }
        }
    };

    // TODO(quintin): clean up shared behavior between this and other types in this file
    template<typename T>
        requires(sizeof(T) % 16 == 0) // Required by WGPU standard, if this does not pass add padding to your type
    struct Uniform {
        T value{};

        wgpu::Device device = nullptr;

        wgpu::Buffer buffer = nullptr;

        auto init(wgpu::Device& device) {
            assert(device);
            assert(!this->device);

            this->device = device;

            wgpu::BufferDescriptor descriptor;
            descriptor.usage = wgpu::BufferUsage::Uniform | wgpu::BufferUsage::CopyDst;
            descriptor.size = sizeof(T);
            buffer = device.createBuffer(descriptor);
        }

        auto enqueueWrite() -> void {
            assert(device);
            assert(buffer);

            device.getQueue().writeBuffer(buffer, 0, std::addressof(value), sizeof(T));
        }

        ~Uniform() {
            if (buffer) {
                buffer.destroy();
                buffer.release();
            }
        }
    };

    struct MeshTexture {
        cv::Mat data;

        wgpu::Texture texture = nullptr;
        wgpu::TextureView view = nullptr;
        wgpu::Sampler sampler = nullptr;

        auto enqueWriteIfUnitialized(wgpu::Device& device) -> bool {
            if (data.empty()) return false;
            if (texture != nullptr) return false;

            // // OpenCV's (0, 0) is in the top-left, but OpenGL's is bottom-left
            // flip(data, data, 0);

            cv::Mat dataToWrite;
            cvtColor(data, dataToWrite, cv::COLOR_BGR2RGBA);

            std::uint32_t mipLevelCount = std::bit_width(static_cast<std::uint32_t>(std::max(dataToWrite.cols, dataToWrite.rows)));

            wgpu::TextureDescriptor descriptor;
            descriptor.dimension = wgpu::TextureDimension::_2D;
            descriptor.size.width = static_cast<std::uint32_t>(dataToWrite.cols);
            descriptor.size.height = static_cast<std::uint32_t>(dataToWrite.rows);
            descriptor.size.depthOrArrayLayers = 1;
            descriptor.mipLevelCount = mipLevelCount;
            descriptor.sampleCount = 1;
            descriptor.format = wgpu::TextureFormat::RGBA8Unorm;
            descriptor.usage = wgpu::TextureUsage::TextureBinding | wgpu::TextureUsage::CopyDst;
            texture = device.createTexture(descriptor);

            wgpu::TextureViewDescriptor viewDescriptor;
            viewDescriptor.arrayLayerCount = 1;
            viewDescriptor.mipLevelCount = descriptor.mipLevelCount;
            viewDescriptor.dimension = wgpu::TextureViewDimension::_2D;
            viewDescriptor.format = descriptor.format;
            view = texture.createView(viewDescriptor);

            wgpu::ImageCopyTexture destination;
            destination.texture = texture;

            wgpu::SamplerDescriptor samplerDescriptor;
            samplerDescriptor.addressModeU = wgpu::AddressMode::Repeat;
            samplerDescriptor.addressModeV = wgpu::AddressMode::Repeat;
            samplerDescriptor.addressModeW = wgpu::AddressMode::Repeat;
            samplerDescriptor.minFilter = wgpu::FilterMode::Linear;
            samplerDescriptor.magFilter = wgpu::FilterMode::Linear;
            samplerDescriptor.mipmapFilter = wgpu::MipmapFilterMode::Linear;
            samplerDescriptor.lodMaxClamp = static_cast<float>(descriptor.mipLevelCount);
            samplerDescriptor.maxAnisotropy = 1;
            sampler = device.createSampler(samplerDescriptor);

            for (; destination.mipLevel < descriptor.mipLevelCount; ++destination.mipLevel) {
                wgpu::TextureDataLayout source;
                source.bytesPerRow = static_cast<std::uint32_t>(dataToWrite.step);
                source.rowsPerImage = static_cast<std::uint32_t>(dataToWrite.rows);
                device.getQueue().writeTexture(
                        destination,
                        dataToWrite.data, dataToWrite.total() * dataToWrite.elemSize(),
                        source,
                        wgpu::Extent3D{static_cast<std::uint32_t>(dataToWrite.cols), static_cast<std::uint32_t>(dataToWrite.rows), 1});

                if (cv::Size nextSize = dataToWrite.size() / 2; nextSize.area() >= 1)
                    cv::resize(dataToWrite, dataToWrite, nextSize, 0, 0);
            }

            return true;
        }

        ~MeshTexture() {
            if (sampler) sampler.release();
            if (view) view.release();
            if (texture) {
                texture.destroy();
                texture.release();
            }
        }
    };

} // namespace mrover
