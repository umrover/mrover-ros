#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::parseParams() -> void {
        XmlRpc::XmlRpcValue objects;
        mNh.getParam("objects", objects);
        if (objects.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::invalid_argument{"objects must be an array"};

        for (int i = 0; i < objects.size(); ++i) { // NOLINT(*-loop-convert)
            XmlRpc::XmlRpcValue const& object = objects[i];

            auto type = xmlRpcValueToTypeOrDefault<std::string>(object, "type");
            auto name = xmlRpcValueToTypeOrDefault<std::string>(object, "name", "<unnamed>");

            NODELET_INFO_STREAM(fmt::format("Loading object: {} of type: {}", name, type));

            if (type == "urdf") {
                if (auto [_, was_added] = mUrdfs.try_emplace(name, *this, object); !was_added) {
                    throw std::invalid_argument{fmt::format("Duplicate object name: {}", name)};
                }
            }
        }
    }

} // namespace mrover
