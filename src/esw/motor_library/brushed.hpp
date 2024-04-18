#pragma once

#include <can_device.hpp>
#include <messaging.hpp>
#include <params_utils.hpp>
#include <std_srvs/Trigger.h>

#include <units/units.hpp>

#include "controller.hpp"

namespace mrover {

    struct Gains {
        double p{}, i{}, d{}, ff{};
    };

    // For now only revolute joints are supported => hardcode to Radians
    class BrushedController final : public ControllerBase<Radians, BrushedController> {
    public:
        BrushedController(ros::NodeHandle const& nh, std::string masterName, std::string controllerName);

        auto setDesiredThrottle(Percent throttle) -> void; // from -1.0 to 1.0

        auto setDesiredPosition(Radians position) -> void;

        auto setDesiredVelocity(RadiansPerSecond velocity) -> void;

        auto adjust(Radians position) -> void;

        auto processCANMessage(CAN::ConstPtr const& msg) -> void;

        auto processMessage(ControllerDataState const& state) -> void;

        auto processMessage(DebugState const&) -> void {}

        auto sendConfiguration() -> void;

        auto getEffort() -> double;

        auto calibrateServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) -> bool;

    private:
        static auto errorToString(BDCMCErrorInfo errorCode) -> std::string;

        bool mIsConfigured{false};
        ConfigCommand mConfigCommand;

        Gains mPositionGains;
        Gains mVelocityGains;
    };

} // namespace mrover
