#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        void onInit() override;
    };

} // namespace mrover
