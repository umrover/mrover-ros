#pragma once

#include "pch.hpp"

namespace mrover {

    constexpr static GLuint GL_INVALID_HANDLE = 0;

    struct Shader {
        GLuint handle = GL_INVALID_HANDLE;

        Shader() = default;

        Shader(Shader&& other) noexcept;
        auto operator=(Shader&& other) noexcept -> Shader&;

        Shader(Shader& other) = delete;
        auto operator=(Shader& other) -> Shader& = delete;

        Shader(std::filesystem::path const& path, GLenum type);

        ~Shader();
    };

    /**
     * \tparam N    Number of shaders in the program.
     *              E.g. two for traditional shader programs (vertex and fragment)
     */
    template<std::size_t N>
    struct Program {
        std::array<Shader, N> shaders;
        std::unordered_map<std::string, GLint> uniforms;

        GLuint handle = GL_INVALID_HANDLE;

        Program() = default;

        Program(Program&& other) noexcept {
            *this = std::move(other);
        }

        auto operator=(Program&& other) noexcept -> Program& {
            if (this != &other) {
                shaders = std::move(other.shaders);
                handle = std::exchange(other.handle, GL_INVALID_HANDLE);
            }
            return *this;
        }

        Program(Program& other) = delete;
        auto operator=(Program& other) -> Program& = delete;

        ~Program() {
            if (handle != GL_INVALID_HANDLE)
                glDeleteProgram(handle);
        }

        explicit Program(std::array<Shader, N>&& shaders) : shaders{std::move(shaders)} {
            handle = glCreateProgram();

            for (auto& shader: this->shaders) {
                glAttachShader(handle, shader.handle);
            }

            glLinkProgram(handle);
            GLint success{};
            glGetProgramiv(handle, GL_LINK_STATUS, &success);
            if (!success) {
                std::array<GLchar, 4096> infoLog{};
                glGetProgramInfoLog(handle, infoLog.size(), nullptr, infoLog.data());
                throw std::runtime_error(std::format("Failed to link program: {}", infoLog.data()));
            }

            ROS_INFO("Successfully created shader program");
        }

        template<typename... Args>
            requires(sizeof...(Args) == N)
        explicit Program(Args&&... args) : Program{std::array<Shader, N>{std::forward<Args>(args)...}} {}

        template<typename T>
        auto uniform(std::string const& name, T const& value) -> void {
            auto it = uniforms.find(name);
            if (it == uniforms.end()) {
                GLint location = glGetUniformLocation(handle, name.c_str());
                if (location == GL_INVALID_VALUE) throw std::runtime_error(std::format("Uniform {} not found", name));
                std::tie(it, std::ignore) = uniforms.emplace(name, location);
            }
            GLuint location = it->second;
            if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int>) {
                glUniform1i(location, value);
            } else {
                glUniform(location, value);
            }
        }
    };

    /**
     * \brief                    Buffer that exists on the CPU and GPU
     * \tparam T                Element type
     * \tparam GlBufferTarget   OpenGL target (e.g. GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER)
     *
     * \note There should be a concept requiring std::is_trivially_copyable_v<T> but for some reason Eigen types are not!
     */
    template<typename T, GLenum GlBufferTarget>
    struct SharedBuffer {
        using value_type = T;

        GLuint handle = GL_INVALID_HANDLE;
        std::vector<T> data;

        auto prepare() -> bool {
            if (data.empty()) return false;
            if (handle != GL_INVALID_HANDLE) return false;

            glGenBuffers(1, &handle);
            assert(handle != GL_INVALID_HANDLE);

            // All subsequent global buffer operations will affect this buffer
            glBindBuffer(GlBufferTarget, handle);
            // Upload to the GPU
            glBufferData(GlBufferTarget, static_cast<GLsizeiptr>(data.size() * sizeof(T)), data.data(), GL_STATIC_DRAW);

            return true;
        }

        ~SharedBuffer() {
            if (handle != GL_INVALID_HANDLE) glDeleteBuffers(1, &handle);
        }
    };

    struct SharedTexture {
        GLuint handle = GL_INVALID_HANDLE;
        cv::Mat data;

        auto prepare() -> bool {
            if (data.empty()) return false;
            if (handle != GL_INVALID_HANDLE) return false;

            // See: https://learnopengl.com/Getting-started/Textures

            // OpenCV's (0, 0) is in the top-left, but OpenGL's is bottom-left
            flip(data, data, 0);

            glGenTextures(1, &handle);
            assert(handle != GL_INVALID_HANDLE);
            glBindTexture(GL_TEXTURE_2D, handle);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // Upload to the GPU
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data.cols, data.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, data.data);
            // Mipmaps make far away textures look better
            glGenerateMipmap(GL_TEXTURE_2D);

            return true;
        }

        ~SharedTexture() {
            if (handle != GL_INVALID_HANDLE) glDeleteTextures(1, &handle);
        }
    };

} // namespace mrover
