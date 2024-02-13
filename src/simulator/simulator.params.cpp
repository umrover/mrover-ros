#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::parseParams() -> void {
        {
            XmlRpc::XmlRpcValue objects;
            mNh.getParam("objects", objects);
            if (objects.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::invalid_argument{"URDFs to load must be an array. Did you rosparam load a simulator config file properly?"};

            for (int i = 0; i < objects.size(); ++i) { // NOLINT(*-loop-convert)
                XmlRpc::XmlRpcValue const& object = objects[i];

                auto type = xmlRpcValueToTypeOrDefault<std::string>(object, "type");
                auto name = xmlRpcValueToTypeOrDefault<std::string>(object, "name", "<unnamed>");

                NODELET_INFO_STREAM(std::format("Loading object: {} of type: {}", name, type));

                if (type == "urdf") {
                    if (auto [_, was_added] = mUrdfs.try_emplace(name, *this, object); !was_added) {
                        throw std::invalid_argument{std::format("Duplicate object name: {}", name)};
                    }
                }
            }
        }
        {
            XmlRpc::XmlRpcValue gpsLinearization;
            mNh.getParam("gps_linearization", gpsLinearization);
            if (gpsLinearization.getType() != XmlRpc::XmlRpcValue::TypeStruct) throw std::invalid_argument{"GPS linearization must be a struct. Did you rosparam load a localization config file properly?"};

            mGpsLinerizationReferencePoint = {
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_latitude"),
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_longitude"),
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_altitude"),
            };
            mGpsLinerizationReferenceHeading = xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_heading");
        }
    }

} // namespace mrover
