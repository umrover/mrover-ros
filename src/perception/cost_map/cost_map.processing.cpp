#include "cost_map.hpp"
#include "../point.hpp"

#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <algorithm>
#include <pstl/glue_execution_defs.h>
#include <se3.hpp>
#include <sensor_msgs/PointCloud2.h>

namespace mrover {

    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;
    }

    /**
     * Stitches local occupancy grid to global occupancy grid
     *
     * @param msg   Local Occupancy Grid Mesasge
     */
    void CostMapNodelet::occupancyGridCallback(nav_msgs::OccupancyGrid const& msg) {

        // Make sure message is valid
        assert(msg.info.width > 0);
        assert(msg.info.height > 0);
        assert(msg.info.resolution > 0);

        // Using tf_tree, get the transformation

        SE3 zed_to_map = SE3::fromTfTree(tf_buffer, "zed2i_left_camera_frame", "map"); //transform straight from zed to map w/o base
        Eigen::Matrix4d transformMatrix = zed_to_map.matrix();

        auto* pointPtr = reinterpret_cast<int const*>(msg.data.data());
        mDimension = static_cast<int>(msg.info.width);
        auto x_origin_local = static_cast<float>(msg.info.origin.position.x);
        auto y_origin_local = static_cast<float>(msg.info.origin.position.y);
        auto resolution_local = static_cast<float>(msg.info.resolution); // m/cell //defined in cost_map.hpp?

        auto x_origin_global = static_cast<float>(mGlobalGridMsg.info.origin.position.x);
        auto y_origin_global = static_cast<float>(mGlobalGridMsg.info.origin.position.y);
        auto resolution_global = static_cast<float>(mGlobalGridMsg.info.resolution); //defined in cost_map.hpp?

        // Do we update load time?
        //globalGridMsg.info.map_load_time

        //For each through full array
        std::for_each(std::execution::par_unseq, &pointPtr, &pointPtr + msg.info.height * msg.info.width, [&](auto point)) {
            // Get x and y relative to the local frame
            int cost = point;
            int i = &point - pointPtr;
            int x_index = i % width;
            int y_index = i / height;
            double x_local = x_origin_local + (resolution_local * x_index);
            double y_local = y_origin_local + (resolution_local * y_index);

            // Convert to homogeneous coordinates
            Eigen::MatrixXd point_local(4, 1);
            point_local(0, 0) = x_local;
            point_local(1, 0) = y_local;
            point_local(2, 0) = 0;
            point_local(3, 0) = 1;

            // Transform using our transformation matrix
            auto xy_global = transformMatrix * point_local;

            // Calculate which index in the global frame the point is in
            double x_global = xy_global(0, 0);
            double y_global = xy_global(1, 0);
            int x_index_global = floor((x_global - x_origin_global) / resolution_global);
            int y_index_global = floor((y_global - y_origin_global) / resolution_global);
            int i_global = (y_index_global * global_dim) + x_index_global;

            // Update our global map by taking the maximum cost at the given point
            int prev_cost = static_cast<int>(globalGridMsg.data[i_global]);
            if (cost > prev_cost) {
                globalGridMsg.data[i_global] = cost;
            }

            // auto* pointPtr = reinterpret_cast<int const*>(msg.data.data());
            // int width = static_cast<int>(msg.info.width);
            // int height = static_cast<int>(msg.info.width);
            // auto x_origin_local = static_cast<float>(msg.info.origin.position.x);
            // auto y_origin_local = static_cast<float>(msg.info.origin.position.y);
            // auto resolution_local = static_cast<float>(msg.info.resolution); // m/cell

            // int width_global = static_cast<int>(globalGridMsg.info.width);
            // int height_global = static_cast<int>(globalGridMsg.info.height);
            // auto x_origin_global = static_cast<float>(globalGridMsg.info.origin.position.x);
            // auto y_origin_global = static_cast<float>(globalGridMsg.info.origin.position.y);
            // auto resolution_global = static_cast<float>(globalGridMsg.info.resolution);

            // auto data_global = static_cast<int8[]>(globalGridMsg.data);

            // // Do we update load time?
            // //globalGridMsg.info.map_load_time

            // //For each through full array
            // std::for_each(std::execution::par_unseq, &pointPtr, &pointPtr + msg.info.height * msg.info.width, [&](auto point) {
            //     // Get x and y relative to the local frame
            //     int cost = point;
            //     int i = &point - pointPtr;
            //     int x_index = i % width;
            //     int y_index = i / height;
            //     double x_local = x_origin_local + (resolution_local * x_index);
            //     double y_local = y_origin_local + (resolution_local * y_index);

            //     // Convert to homogeneous coordinates
            //     Eigen::MatrixXd point_local(4, 1);
            //     point_local(0, 0) = x_local;
            //     point_local(1, 0) = y_local;
            //     point_local(2, 0) = 0;
            //     point_local(3, 0) = 1;

            //     // Transform using our transformation matrix
            //     auto xy_global = transformMatrix * point_local;

            //     // Calculate which index in the global frame the point is in
            //     double x_global = xy_global(0, 0);
            //     double y_global = xy_global(1, 0);
            //     int x_index_global = floor((x_global - x_origin_global) / resolution_global);
            //     int y_index_global = floor((y_global - y_origin_global) / resolution_global);
            //     int i_global = (y_index_global * width_global) + x_index_global;

            //     // Update our global map by taking the maximum cost at the given point
            //     //int prev
            // });
        }
    }
} // namespace mrover