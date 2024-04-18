#pragma once

#include "pch.hpp"

namespace mrover {

    class ClickIkNodelet final : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;
        ros::Subscriber mPcSub;
        ros::Publisher mIkPub;
        ros::Subscriber mStatusSub;
        actionlib::SimpleActionServer<mrover::ClickIkAction> server = actionlib::SimpleActionServer<mrover::ClickIkAction>(mNh, "do_click_ik", false);

        IK message;
        ros::Timer timer;

        Point const* mPoints{};
        std::size_t mNumPoints{};
        std::size_t mPointCloudWidth{};
        std::size_t mPointCloudHeight{};

        tf2_ros::Buffer mTfBuffer{};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

    public:
        ClickIkNodelet() = default;

        ~ClickIkNodelet() override = default;

        void onInit() override;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
        void statusCallback(ArmStatus const&);

        void startClickIk();
        void cancelClickIk();

        //Taken line for line from percep object detection code
        auto spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<Point>;
    };

} // namespace mrover
