#pragma once

#include "pch.hpp"

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
        wgpu::Buffer handle = nullptr;

        auto prepare() -> bool {
            if (data.empty()) return false;
            // if (handle != GL_INVALID_HANDLE) return false;

            // glGenBuffers(1, &handle);
            // assert(handle != GL_INVALID_HANDLE);

            // // All subsequent global buffer operations will affect this buffer
            // glBindBuffer(GlBufferTarget, handle);
            // // Upload to the GPU
            // glBufferData(GlBufferTarget, static_cast<GLsizeiptr>(data.size() * sizeof(T)), data.data(), GL_STATIC_DRAW);

            return true;
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
