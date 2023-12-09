#include "simulator.hpp"

namespace mrover {

    auto readTextFile(std::filesystem::path const& path) -> std::string {
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        file.open(path);
        return {std::istreambuf_iterator{file}, std::istreambuf_iterator<char>{}};
    }

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

    Shader::Shader(Shader&& other) noexcept
        : handle{std::exchange(other.handle, GL_INVALID_HANDLE)} {}

    Shader::~Shader() {
        if (handle == GL_INVALID_HANDLE) return;

        glDeleteShader(handle);
    }

    Program::Program(Shader&& vertexShader, Shader&& fragmentShader)
        : vertexShader{std::move(vertexShader)}, fragmentShader{std::move(fragmentShader)} {

        handle = glCreateProgram();

        glAttachShader(handle, this->vertexShader.handle);
        glAttachShader(handle, this->fragmentShader.handle);

        glLinkProgram(handle);
        GLint success{};
        glGetProgramiv(handle, GL_LINK_STATUS, &success);
        if (!success) {
            std::array<GLchar, 4096> infoLog{};
            glGetProgramInfoLog(handle, infoLog.size(), nullptr, infoLog.data());
            throw std::runtime_error(std::format("Failed to link program: {}", infoLog.data()));
        }
    }

    Program::~Program() {
        if (handle == GL_INVALID_HANDLE) return;

        glDeleteProgram(handle);
    }
}
