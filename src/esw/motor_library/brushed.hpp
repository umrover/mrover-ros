#pragma once

#include <can_manager.hpp>
#include <controller.hpp>

namespace mrover {

    class BrushedController : public Controller {
    public:
        void update(std::span<std::byte const> frame) override;

        void set_desired_throttle(Dimensionless throttle) override; // from -1.0 to 1.0
        void set_desired_velocity(RadiansPerSecond velocity) override;
        void set_desired_position(Radians position) override;

        BrushedController(ros::NodeHandle& nh, const std::string& name) : Controller(nh, name) {}
        ~BrushedController() override = default;

    private:
    };

} // namespace mrover
