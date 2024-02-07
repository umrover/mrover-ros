#include "simulator.hpp"

namespace mrover {

    constexpr static auto PACKAGE_URI_PREFIX = "package://"sv;
    constexpr static auto FILE_URI_PREFIX = "file://"sv;
    constexpr static auto TEXTURE_URI_PREFIX = "package://mrover/urdf/textures"sv;

    auto performXacro(std::filesystem::path const& path) -> std::string {
        // "xacro" is a Python library so unfortunately we have to run it as a subprocess

        std::string command = std::format("xacro {}", path.string());
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
        if (!pipe) throw std::runtime_error{std::format("Failed to xacro: {}", path.string())};

        std::array<char, 128> chunk{};
        std::string output;
        while (std::fgets(chunk.data(), chunk.size(), pipe.get()) != nullptr) output += chunk.data();

        return output;
    }

    auto readTexture(std::filesystem::path const& textureFileName) -> cv::Mat {
        if (textureFileName.has_parent_path()) throw std::invalid_argument{std::format("Must be only a filename: {}", textureFileName.string())};

        std::string textureUri = std::format("{}/{}", TEXTURE_URI_PREFIX, textureFileName.string());
        cv::Mat texture = imread(uriToPath(textureUri), cv::IMREAD_COLOR);
        if (texture.empty()) throw std::runtime_error{std::format("Failed to load texture: {}", textureUri)};

        return texture;
    }

    auto readTextFile(std::filesystem::path const& path) -> std::string {
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        file.open(path);
        return {std::istreambuf_iterator{file}, std::istreambuf_iterator<char>{}};
    }

    auto uriToPath(std::string_view uri) -> std::filesystem::path {
        if (uri.starts_with(PACKAGE_URI_PREFIX)) {
            uri.remove_prefix(PACKAGE_URI_PREFIX.size());
        } else if (uri.starts_with(FILE_URI_PREFIX)) {
            uri.remove_prefix(FILE_URI_PREFIX.size());
        } else {
            throw std::invalid_argument{std::format("Unsupported URI prefix: {}", uri)};
        }

        std::filesystem::path path{uri};
        std::filesystem::path package = *path.begin();
        std::filesystem::path rest = path.lexically_relative(package);

        std::filesystem::path packagePath = ros::package::getPath(package);
        if (packagePath.empty()) throw std::runtime_error{std::format("Failed to find package: {}", package.string())};

        return packagePath / rest;
    }

} // namespace mrover
