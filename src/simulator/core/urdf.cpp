#include "simulator.hpp"

namespace mrover {

    URDF::URDF(XmlRpc::XmlRpcValue const& init) {
        auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
        if (!model.initParam(paramName)) throw std::runtime_error{std::format("Failed to parse URDF from param: {}", paramName)};

        for (auto const& [link_name, link]: model.links_) {
            if (!link->visual || !link->visual->geometry) continue;

            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
            if (!mesh) continue;

            std::string uri = mesh->filename;
            uriToMeshes[uri].emplace_back(uri);
        }
    }

}
