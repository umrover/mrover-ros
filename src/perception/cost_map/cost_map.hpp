#include "nav_msgs/OccupancyGrid.h"
#include "pch.hpp"

namespace mrover {

    class CostMapNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;
        void onInit() override;

        bool publishCostMaps = false; // Do we publish our global cost maps?
        bool verbose = false;         // Do we print extra information for debugging?
        float resolution = 1;         // In m/cell
        int cell_width = 1;            // Number of cells wide
        int cell_height = 1;           // Number of cells high
        int global_dim = 15;
        const tf2_ros::Buffer tf_buffer = tf2_ros::Buffer();

        nav_msgs::OccupancyGrid globalGridMsg;


    public:
        CostMapNodelet() = default;
        ~CostMapNodelet() override = default;
        void occupancyGridCallback(nav_msgs::OccupancyGrid const& msg);
    };
} // namespace mrover