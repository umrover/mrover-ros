#pragma once

#include "controller.hpp"

class BrushedController : public Controller {
public:
    void update(uint64_t frame) override;

    void set_desired_throttle(double throttle) override; // from -1.0 to 1.0
    void set_desired_velocity(double velocity) override; // from -1.0 to 1.0
    void set_desired_position(double position) override;
    MotorType get_type() override;

    BrushedController(ros::NodeHandle& n, const std::string& name) : Controller(n, name) {}
    ~BrushedController() override = default;

private:
};