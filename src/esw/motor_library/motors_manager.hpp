#pragma once

#include "controller.hpp"

#include <map>

class MotorsManager {
public:
    auto& get_controller(std::string const& name) {
        return m.at(name);
    }

    virtual void process_frame(uint64_t frame) = 0;

private:
    std::map<std::string, std::unique_ptr<Controller>> m;
};