#include "simulator.hpp"

namespace mrover {

    constexpr auto PACKAGE_URI_PREFIX = "package://"sv;
    constexpr auto FILE_URI_PREFIX = "file://"sv;
    constexpr auto XACRO_BINARY_PATH = "/opt/ros/noetic/bin/xacro"sv;

    auto performXacro(std::filesystem::path const& path) -> std::string {
        // xacro is a Python library so unfortunately we have to run it as a subprocess
        std::string output;
        boost::process::ipstream is;
        boost::process::child c{std::format("{} {}", XACRO_BINARY_PATH, path.string()), boost::process::std_out > is};
        std::string line;
        while (c.running() && std::getline(is, line) && !line.empty()) {
            output += line + '\n';
        }
        c.wait();
        if (c.exit_code()) throw std::runtime_error{std::format("Failed to xacro: {}", output)};

        return output;
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

        std::string_view package = uri.substr(0, uri.find('/'));
        std::string_view path = uri.substr(package.size() + 1);

        rospack::Rospack rp;
        std::string packagePath;
        rp.find(package.data(), packagePath);

        return std::filesystem::path{packagePath} / path;
    }

} // namespace mrover