#pragma once

#include <can_manager.hpp>
#include <controller.hpp>

class BrushedController : public Controller {
public:
    void update(std::span<std::byte const> frame) override;

    void set_desired_throttle(float throttle) override; // from -1.0 to 1.0
    void set_desired_velocity(float velocity) override;
    void set_desired_position(float position) override;

    BrushedController(ros::NodeHandle& nh, const std::string& name) : Controller(nh, name) {}
    ~BrushedController() override = default;

private:
};