#pragma once

#include "controller.hpp"


class BrushedController : public Controller {
public:
    void update(uint64_t frame);

    void set_desired_throttle(double throttle); // from -1.0 to 1.0
    void set_desired_velocity(double velocity); // from -1.0 to 1.0
    void set_desired_position(double position);
    MotorType get_type();

    BrushedController(ros::NodeHandle* n, const std::string &name) : Controller(n, name) {}
    ~BrushedController() = default;

private:

};