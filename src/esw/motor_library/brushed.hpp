#pragma once

#include "messaging.hpp"
#include "params_utils.hpp"
#include <can_device.hpp>
#include <controller.hpp>

namespace mrover {

    struct Gains {
        double p{}, i{}, d{}, ff{};
    };

    class BrushedController : public Controller {
    public:
        void setDesiredThrottle(Percent throttle) override; // from -1.0 to 1.0
        void setDesiredVelocity(RadiansPerSecond velocity) override;
        void setDesiredPosition(Radians position) override;

        void processCANMessage(CAN::ConstPtr const& msg) override;

        void processMessage(ControllerDataState const& state);

        void processMessage(DebugState const&) {}

        void sendConfiguration();

        double getEffort() override;

        BrushedController(ros::NodeHandle const& nh, std::string name, std::string controllerName);
        ~BrushedController() override = default;

    private:
        static std::string errorToString(BDCMCErrorInfo errorCode);

        bool mIsConfigured = false;
        ConfigCommand mConfigCommand;

        Gains mPositionGains;
        Gains mVelocityGains;
    };

} // namespace mrover
