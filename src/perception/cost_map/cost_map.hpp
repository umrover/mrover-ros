#include "pch.hpp"

namespace mrover {

    struct CostMapNode {
        cv::Point2f position{};
        double cost = 0;
    };
    /*
    (2,3) -> 100
    (3,4) -> 0
    (1,1) -> 100
    
    -1 -1 -1 -1 -1
    -1 100 -1 -1 -1
    -1 -1 -1 100 -1
    -1 -1 -1 -1 0
    -1 -1 -1 -1 -1
    
    */

    class CostMapNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;
        std::vector<CostMapNode> costMapPointList;
        void onInit() override;

        bool publishCostMaps = false;
        bool verbose = false;
        float element_size = 1;
        int map_width = 1;
        int map_height = 1;
        float cutoff = 1;

    public:
        CostMapNodelet() = default;
        ~CostMapNodelet() override = default;
        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

    };
}