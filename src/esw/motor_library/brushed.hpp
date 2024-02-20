#pragma once

#include <can_device.hpp>
#include <messaging.hpp>
#include <params_utils.hpp>
#include <std_srvs/Trigger.h>

#include "controller.hpp"

namespace mrover {

    struct Gains {
        double p{}, i{}, d{}, ff{};
    };

    class BrushedController : public Controller {
    public:
        void setDesiredThrottle(Percent throttle) override; // from -1.0 to 1.0
        void setDesiredVelocity(RadiansPerSecond velocity) override;
        void setDesiredPosition(Radians position) override;
        void adjust(Radians position) override;

        void processCANMessage(CAN::ConstPtr const& msg) override;

        void processMessage(ControllerDataState const& state);

        void processMessage(DebugState const&) {}

        void sendConfiguration();

        double getEffort() override;

        BrushedController(ros::NodeHandle const& nh, std::string name, std::string controllerName);
        ~BrushedController() override = default;

        bool calibrateServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    private:
        static std::string errorToString(BDCMCErrorInfo errorCode);

        bool mIsConfigured = false;
        ConfigCommand mConfigCommand;

        Gains mPositionGains;
        Gains mVelocityGains;
    };

} // namespace mrover
