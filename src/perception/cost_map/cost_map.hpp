#include "pch.hpp"

namespace mrover {


    class CostMapNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;
        void onInit() override;

        bool mPublishCostMap{}; // If set, publish the global costmap
        float mResolution{};    // Cells per meter
        int mDimension{};       // Dimensions of the square costmap in meters
        float mNormalThreshold = 0.5;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener{tf_buffer};

        std::optional<SE3> mPreviousPose;
        nav_msgs::OccupancyGrid mGlobalGridMsg;


    public:
        CostMapNodelet() = default;

        ~CostMapNodelet() override = default;

        void configCallback();
        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);
    };
} // namespace mrover