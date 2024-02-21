#include "long_range_tag_detector.hpp"
#include "mrover/LongRangeTags.h"
#include <vector>

namespace mrover {

    auto LongRangeTagDetectorNodelet::specificOnInit() -> void {
        mPnh.param<float>("long_range_fov", mLongRangeFov, 80.0);

            mImgPub = mNh.advertise<sensor_msgs::Image>("long_range_tag_detection", 1);
            mLongRangeTagsPub = mNh.advertise<LongRangeTags>("tags", 1);

            mImgSub = mNh.subscribe("long_range_image", 1, &LongRangeTagDetectorNodelet::imageCallback, this);

} // namespace mrover

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeTagDetectorNodelet, nodelet::Nodelet)
}