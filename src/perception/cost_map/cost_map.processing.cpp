#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;

        std::uint32_t maxNumPoints = msg->height * msg->width / mDownSamplingFactor / mDownSamplingFactor;
        mPointsInMap.clear();
        mPointsInMap.reserve(maxNumPoints);

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();
            auto* points = reinterpret_cast<Point const*>(msg->data.data());

            // std::vector<Eigen::Index> rowIndices;
            // rowIndices.resize(msg->width / mDownSamplingFactor);
            // std::ranges::generate(rowIndices, [&, c = 0]() mutable { return c++; });
            //
            // for (Eigen::Index r = 0; r < msg->height / mDownSamplingFactor; r++) {
            //     std::for_each(std::execution::par_unseq, rowIndices.begin(), rowIndices.end(), [&](Eigen::Index c) {
            //         auto* point = points + r * mDownSamplingFactor * msg->width + c * mDownSamplingFactor;
            //         R3f pointInCamera{point->x, point->y, point->z};
            //         R3f normalInCamera{point->normal_x, point->normal_y, point->normal_z};
            //
            //         Eigen::Index i = r * msg->width / mDownSamplingFactor + c;
            //         mPointsInMap.col(i) = cameraToMap.act(pointInCamera);
            //         mNormalsInMap.col(i) = cameraToMap.asSO3().act(normalInCamera); // Normal is a direction as should not be affected by translation
            //     });
            // }

            for (Eigen::Index r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (Eigen::Index c = 0; c < msg->width; c += mDownSamplingFactor) {
                    auto* point = points + r * msg->width + c;

                    // Discard points too close to the camera
                    if (point->x < 1) continue;

                    R3f pointInCamera{point->x, point->y, point->z};
                    R3f normalInCamera{point->normal_x, point->normal_y, point->normal_z};

                    // Normal is a direction as should not be affected by translation
                    mPointsInMap.emplace_back(cameraToMap.act(pointInCamera), cameraToMap.asSO3().act(normalInCamera));
                }
            }

            for (auto const& [pointInMap, normalInMap]: mPointsInMap) {
                double xInMap = pointInMap.x();
                double yInMap = pointInMap.y();

                if (xInMap < mGlobalGridMsg.info.origin.position.x || xInMap > mGlobalGridMsg.info.origin.position.x + mDimension ||
                    yInMap < mGlobalGridMsg.info.origin.position.y || yInMap > mGlobalGridMsg.info.origin.position.y + mDimension) continue;

                int xIndex = std::floor((xInMap - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                int yIndex = std::floor((yInMap - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                auto costMapIndex = mGlobalGridMsg.info.width * yIndex + xIndex;

                // Z is the vertical component of the normal
                // A small Z component indicates largely horizontal normal (surface is vertical)
                std::int8_t cost = normalInMap.z() < mNormalThreshold ? OCCUPIED_COST : FREE_COST;

                // Low pass filter
                constexpr double alpha = 0.1;
                mGlobalGridMsg.data[costMapIndex] = static_cast<std::int8_t>(alpha * cost + (1 - alpha) * mGlobalGridMsg.data[costMapIndex]);
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (tf2::TransformException const& e) {
            ROS_WARN_STREAM(std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

} // namespace mrover