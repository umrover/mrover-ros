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

        ROS_INFO("Successfully created shader program");
    }

    Program::Program(Program&& other) noexcept {
        *this = std::move(other);
    }

    auto Program::operator=(Program&& other) noexcept -> Program& {
        if (this != &other) {
            vertexShader = std::move(other.vertexShader);
            fragmentShader = std::move(other.fragmentShader);
            handle = std::exchange(other.handle, GL_INVALID_HANDLE);
        }
        return *this;
    }

    Program::~Program() {
        if (handle != GL_INVALID_HANDLE)
            glDeleteProgram(handle);
    }
} // namespace mrover
