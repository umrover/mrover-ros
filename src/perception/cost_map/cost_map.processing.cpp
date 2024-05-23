#include "cost_map.hpp"

namespace mrover {

    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto square(auto x) -> auto { return x * x; }

    auto mapToGrid(Eigen::Vector3f const& positionInMap, nav_msgs::OccupancyGrid const& grid) -> long {
        Eigen::Vector2f origin{grid.info.origin.position.x, grid.info.origin.position.y};
        Eigen::Vector2f gridFloat = (positionInMap.head<2>() - origin) / grid.info.resolution;
        long gridX = std::floor(gridFloat.x());
        long gridY = std::floor(gridFloat.y());
        return gridY * grid.info.width + gridX;
    }

    auto CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();

            struct BinEntry {
                R3f pointInCamera;
                R3f pointInMap;
            };
            using Bin = std::vector<BinEntry>;

            std::vector<Bin> bins;
            bins.resize(mGlobalGridMsg.data.size());

            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            for (std::size_t r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (std::size_t c = 0; c < msg->width; c += mDownSamplingFactor) {
                    Point const& point = points[r * msg->width + c];
                    R3f pointInCamera{point.x, point.y, point.z};

                    // Points with no stereo correspondence are NaN's, so ignore them
                    if (pointInCamera.hasNaN()) continue;

                    if (double distanceSquared = pointInCamera.squaredNorm();
                        distanceSquared < square(mNearClip) || distanceSquared > square(mFarClip)) continue;

                    R3f pointInMap = cameraToMap.act(pointInCamera);

                    long index = mapToGrid(pointInMap, mGlobalGridMsg);
                    if (index < 0 || index >= static_cast<long>(mGlobalGridMsg.data.size())) continue;

                    bins[index].emplace_back(BinEntry{pointInCamera, pointInMap});
                }
            }

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
                Bin& bin = bins[i];
                if (bin.size() < 16) continue;

                std::size_t pointsHigh = std::ranges::count_if(bin, [](BinEntry const& entry) {
                    return entry.pointInCamera.z() > 0;
                });
                double percent = static_cast<double>(pointsHigh) / static_cast<double>(bin.size());

                std::int8_t cost = percent > 0.25 ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                constexpr double alpha = 0.05;
                cell = static_cast<std::int8_t>(alpha * cost + (1 - alpha) * cell);
            }

            mCostMapPub.publish(mGlobalGridMsg);
        } catch (tf2::TransformException const& e) {
            ROS_WARN_STREAM_THROTTLE(1, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    auto CostMapNodelet::moveCostMapCallback(MoveCostMap::Request& req, MoveCostMap::Response& res) -> bool {
        SE3d centerInMap = SE3Conversions::fromTfTree(mTfBuffer, req.course, mMapFrame);
        std::ranges::fill(mGlobalGridMsg.data, UNKNOWN_COST);
        mGlobalGridMsg.info.origin.position.x = centerInMap.x() - mSize / 2;
        mGlobalGridMsg.info.origin.position.y = centerInMap.y() - mSize / 2;
        return res.success = true;
    }

} // namespace mrover
