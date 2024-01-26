#include "cost_map.hpp"
#include "../point.hpp"

#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <pstl/glue_execution_defs.h>

namespace mrover {

    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;
        try {
            SE3 zed_to_map = SE3::fromTfTree(tf_buffer, "zed2i_left_camera_frame", "map");
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            // std::for_each(points, points + msg->width * msg->height, [&](auto& point) {
            for (auto point = points; point - points < msg->width * msg->height; point++) {
                SE3 point_in_zed{R3{point->x, point->y, 0.0}, {}};
                SE3 point_in_map = zed_to_map * point_in_zed;

                double x = point_in_map.position().x();
                double y = point_in_map.position().y();
                int x_index = floor((x - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                int y_index = floor((y - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                auto i = mGlobalGridMsg.info.width * y_index + x_index;

                R3 normal{point->normal_x, point->normal_y, point->normal_z};
                normal.normalize();
                // get vertical component of (unit) normal vector
                double z_comp = normal.z();
                // small z component indicates largely horizontal normal (surface is vertical)
                signed char cost = z_comp < mNormalThreshold ? 100 : 0;

                mGlobalGridMsg.data[i] = std::max(mGlobalGridMsg.data[i], cost);
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
        }
    }
} // namespace mrover
