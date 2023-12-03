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

        SE3 zed_to_map = SE3::fromTfTree(tf_buffer, "zed2i_left_camera_frame", "map");

        auto* points = reinterpret_cast<const Point*>(msg->data.data());
        std::for_each(points, points + msg->width * msg->height, [&](auto& point) {
            SE3 point_in_zed{R3{point.x, point.y, 0.0}};
            SE3 point_in_map = zed_to_map * point_in_zed;

            int x_index = floor((point_in_map.position().x() - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
            int y_index = floor((point_in_map.position().y() - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
            auto i = mGlobalGridMsg.info.width * y_index + x_index;

            // dot unit normal with <0, 0, 1>
            R3 normal{point.normal_x, point.normal_y, point.normal_z};
            normal.normalize();
            double dot_product = normal.z();
            signed char cost = dot_product < mNormalThreshold ? 100 : 0;

            mGlobalGridMsg.data[i] = std::max(mGlobalGridMsg.data[i], cost);
        });
        mCostMapPub.publish(mGlobalGridMsg);
    }
} // namespace mrover