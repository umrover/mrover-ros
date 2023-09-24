#pragma once

#include "controller.hpp"


class BrushedController final : public Controller {
public:
    void update_motor(float speed, float position) override;

private:

};