#include "cost_map.hpp"

namespace mrover {

    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;

        try {
            SE3d zed_to_map = SE3Conversions::fromTfTree(mTfBuffer, "map", "zed_left_camera_frame");
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            // std::for_each(points, points + msg->width * msg->height, [&](auto* point) {
            for (auto point = points; point - points < msg->width * msg->height; point += mDownSamplingFactor) {
                Eigen::Vector4d p{point->x, point->y, point->z, 1};
                point_matrix.col((point - points) / mDownSamplingFactor) = p;
                Eigen::Vector4d normal{point->normal_x, point->normal_y, point->normal_z, 0};
                normal_matrix.col((point - points) / mDownSamplingFactor) = normal;
            }

            // TODO(neven): Make sure this still works with manif
            point_matrix = zed_to_map.transform() * point_matrix;
            normal_matrix = zed_to_map.transform() * normal_matrix;

            for (uint32_t i = 0; i < mNumPoints; i++) {
                double x = point_matrix(0, i);
                double y = point_matrix(1, i);
                Eigen::Vector4d n = normal_matrix.col(i);

                if (x >= mGlobalGridMsg.info.origin.position.x && x <= mGlobalGridMsg.info.origin.position.x + mDimension &&
                    y >= mGlobalGridMsg.info.origin.position.y && y <= mGlobalGridMsg.info.origin.position.y + mDimension) {
                    int x_index = std::floor((x - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                    int y_index = std::floor((y - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                    auto ind = mGlobalGridMsg.info.width * y_index + x_index;

                    Eigen::Vector3d normal{n.x(), n.y(), n.z()};
                    // normal.normalize();
                    // get vertical component of (unit) normal vector
                    double z_comp = normal.z();
                    // small z component indicates largely horizontal normal (surface is vertical)
                    signed char cost = z_comp < mNormalThreshold ? 1 : 0;

                    mGlobalGridMsg.data[ind] = std::max(mGlobalGridMsg.data[ind], cost);
                }
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
            // TODO(neven): Catch TF exceptions only, and log warn them
        }
    }

} // namespace mrover
