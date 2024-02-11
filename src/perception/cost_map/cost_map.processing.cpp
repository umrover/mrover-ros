#include "cost_map.hpp"
#include "../point.hpp"

#include "loop_profiler.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <cstdint>
#include <pstl/glue_execution_defs.h>
#include <regex>

namespace mrover {

    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;
        try {
            SE3 zed_to_map = SE3::fromTfTree(tf_buffer, "odom", "zed_left_camera_frame");
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            int step = 4;
            uint32_t num_points = msg->width * msg->height / step;
            Eigen::MatrixXd point_matrix(4, num_points);
            Eigen::MatrixXd normal_matrix(4, num_points);
            // std::for_each(points, points + msg->width * msg->height, [&](auto* point) {
            for (auto point = points; point - points < msg->width * msg->height; point += step) {
                Eigen::Vector4d p{point->x, point->y, point->z, 1};
                point_matrix.col((point - points) / step) = p;
                Eigen::Vector4d normal{point->normal_x, point->normal_y, point->normal_z, 0};
                normal_matrix.col((point - points) / step) = normal;
            }

            point_matrix = zed_to_map.matrix() * point_matrix;
            normal_matrix = zed_to_map.matrix() * normal_matrix;

            for (uint32_t i = 0; i < num_points; i++) {
                double x = point_matrix(0, i);
                double y = point_matrix(1, i);
                double z = point_matrix(2, i);
                Eigen::Vector4d n = normal_matrix.col(i);

                if (x >= -1 * mDimension / 2 && x <= mDimension / 2 &&
                    y >= -1 * mDimension / 2 && y <= mDimension / 2 &&
                    z < 2) {
                    int x_index = floor((x - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                    int y_index = floor((y - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                    auto ind = mGlobalGridMsg.info.width * y_index + x_index;

                    Eigen::Vector3d normal{n.x(), n.y(), n.z()};
                    normal.normalize();
                    // get vertical component of (unit) normal vector
                    double z_comp = normal.z();
                    // small z component indicates largely horizontal normal (surface is vertical)
                    signed char cost = z_comp < 0.5 ? 1 : 0;
                    // signed char cost = 0;
                    // if (z_comp < 0.2)
                    //     cost = 1;
                    // else if (z_comp < 0.4)
                    //     cost = 10;
                    // else if (z_comp < 0.6)
                    //     cost = 20;

                    mGlobalGridMsg.data[ind] = std::max(mGlobalGridMsg.data[ind], cost);
                }
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
        }
    }
} // namespace mrover
