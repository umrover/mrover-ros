#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace mrover {


    class CostMapNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;
        void onInit() override;

        bool mPublishCostMap{}; // If set, publish the global costmap
        float mResolution{};    // Meters per cell
        float mDimension{};     // Dimensions of the square costmap in meters
        float mNormalThreshold = 0.9;
        int mDownSamplingFactor = 4;
        uint32_t mNumPoints = 640 * 480 / mDownSamplingFactor;
        Eigen::MatrixXd point_matrix{4, mNumPoints};
        Eigen::MatrixXd normal_matrix{4, mNumPoints};

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