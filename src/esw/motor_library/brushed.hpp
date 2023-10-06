#pragma once

#include "controller.hpp"
#include "can_manager.hpp"

class BrushedController : public Controller {
public:
    void update(const std::vector<uint8_t> &frame) override;

    void set_desired_throttle(float throttle) override; // from -1.0 to 1.0
    void set_desired_velocity(float velocity) override;
    void set_desired_position(float position) override;

    BrushedController(ros::NodeHandle& n, const std::string& name) : Controller(n, name) {}
    ~BrushedController() override = default;

private:
};