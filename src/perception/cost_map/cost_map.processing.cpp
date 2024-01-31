#include "cost_map.hpp"
#include "../point.hpp"

#include "loop_profiler.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <pstl/glue_execution_defs.h>
#include <regex>

namespace mrover {

    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;
        try {
            SE3 zed_to_map = SE3::fromTfTree(tf_buffer, "map", "zed2i_left_camera_frame");
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            // std::for_each(points, points + msg->width * msg->height, [&](auto* point) {
            for (auto point = points; point - points < msg->width * msg->height; point++) {
                Eigen::Vector4d point_in_map = zed_to_map.matrix() * Eigen::Vector4d{point->x, point->y, 0, 1};

                double x = point_in_map.x();
                double y = point_in_map.y();
                if (x >= -1 * mDimension / 2 && x <= mDimension / 2 &&
                    y >= -1 * mDimension / 2 && y <= mDimension / 2) {
                    int x_index = floor((x - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                    int y_index = floor((y - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                    auto i = mGlobalGridMsg.info.width * y_index + x_index;

                    Eigen::Vector3d normal_in_zed{point->normal_x, point->normal_y, point->normal_z};
                    Eigen::Vector3d normal_in_map = zed_to_map.rotation() * normal_in_zed;
                    normal_in_map.normalize();
                    // get vertical component of (unit) normal vector
                    double z_comp = normal_in_map.z();
                    // small z component indicates largely horizontal normal (surface is vertical)
                    signed char cost = z_comp < mNormalThreshold ? 100 : 0;

                    mGlobalGridMsg.data[i] = std::max(mGlobalGridMsg.data[i], cost);
                }
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
        }
    }
} // namespace mrover
