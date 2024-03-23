#pragma once

#include "pch.hpp"

namespace mrover{
    class ExceptionHandler : public nodelet::Nodelet{
    private:
        ros::NodeHandle mNh, mPnh;

        auto onInit() -> void override;

    public:
        ExceptionHandler() = default;
        
        ~ExceptionHandler() override = default;
    };
};
