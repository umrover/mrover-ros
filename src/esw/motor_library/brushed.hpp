#pragma once

#include "controller.hpp"


class BrushedController final : public Controller {
public:
    void update(uint64_t frame) override;

    void set_desired_speed(double velocity) override;
    void set_desired_position(int position) override;

private:

};