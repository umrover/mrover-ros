#pragma once

#include "controller.hpp"


class BrushedController final : public Controller {
public:
    void update(uint64_t frame) override;

    void set_desired_throttle(double throttle); // from -1.0 to 1.0
    void set_desired_velocity(double velocity); // from -1.0 to 1.0
    void set_desired_position(double position) override;
    MotorType get_type() override;

private:

};