#pragma once

#include "controller.hpp"


class BrushedController final : public Controller {
public:
    void update(uint64_t frame) override;

    void set_desired_speed_unit(double speed); // from -1.0 to 1.0
    void set_desired_position(int position) override;
    MotorType get_type() override;

private:

};