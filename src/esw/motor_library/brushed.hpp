#pragma once

#include <can_manager.hpp>
#include <controller.hpp>

#include "messaging.hpp"

namespace mrover {

    class BrushedController : public Controller {
    public:
        void set_desired_throttle(Percent throttle) override; // from -1.0 to 1.0
        void set_desired_velocity(RadiansPerSecond velocity) override;
        void set_desired_position(Radians position) override;

        void send_configuration();

        BrushedController(ros::NodeHandle& nh, std::string const& name, std::string const& controller_name);
        ~BrushedController() override = default;

    private:
        bool m_is_configured = false;
        ConfigCommand m_config_command;
    };

} // namespace mrover
