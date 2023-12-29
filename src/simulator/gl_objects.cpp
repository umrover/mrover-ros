#include "simulator.hpp"

namespace mrover {

    Shader::Shader(std::filesystem::path const& path, GLenum type) {
        handle = glCreateShader(type);

        std::string source = readTextFile(path);
        char const* sourceCString = source.c_str();
        glShaderSource(handle, 1, &sourceCString, nullptr);

        glCompileShader(handle);
        GLint success{};
        glGetShaderiv(handle, GL_COMPILE_STATUS, &success);
        if (!success) {
            std::array<GLchar, 4096> infoLog{};
            glGetShaderInfoLog(handle, infoLog.size(), nullptr, infoLog.data());
            throw std::runtime_error(std::format("Failed to compile shader {}: {}", path.c_str(), infoLog.data()));
        }
    }

    Shader::Shader(Shader&& other) noexcept {
        *this = std::move(other);
    }

    auto Shader::operator=(Shader&& other) noexcept -> Shader& {
        if (this != &other) {
            handle = std::exchange(other.handle, GL_INVALID_HANDLE);
        }
        return *this;
    }

    Shader::~Shader() {
        if (handle != GL_INVALID_HANDLE)
            glDeleteShader(handle);
    }

} // namespace mrover
