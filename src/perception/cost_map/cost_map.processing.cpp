#include "cost_map.hpp"

namespace mrover {

    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto mapToGrid(Eigen::Vector2f const& positionInMap, nav_msgs::OccupancyGrid const& grid) -> Eigen::Vector2i {
        Eigen::Vector2f origin{grid.info.origin.position.x, grid.info.origin.position.y};
        Eigen::Vector2f gridFloat = (positionInMap - origin) / grid.info.resolution;
        return {std::lround(gridFloat.x()), std::lround(gridFloat.y())};
    }

    auto CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;
        
        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map", msg->header.stamp).cast<float>();
            auto* points = reinterpret_cast<Point const*>(msg->data.data());

            for (Eigen::Index r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (Eigen::Index c = 0; c < msg->width; c += mDownSamplingFactor) {
                    Point const& point = points[r * msg->width + c];
                    R3f pointInCamera{point.x, point.y, point.z};

                    // Points with no stereo correspondence are NaN's, so ignore them
                    if (pointInCamera.hasNaN()) continue;

                    if (double distSq = pointInCamera.squaredNorm(); distSq < 1 * 1 || distSq > 8 * 8) continue;

                    
                }
            }

            mCostMapPub.publish(mGlobalGridMsg);
        } catch (tf2::TransformException const& e) {
            ROS_WARN_STREAM_THROTTLE(1, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    auto CostMapNodelet::moveCostMapCallback(MoveCostMap::Request& req, MoveCostMap::Response& res) -> bool {
        SE3d waypointPos = SE3Conversions::fromTfTree(mTfBuffer, req.course, mWorldFrame);
        std::ranges::fill(mGlobalGridMsg.data, UNKNOWN_COST);
        mGlobalGridMsg.info.origin.position.x = waypointPos.x() - mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = waypointPos.y() - mDimension / 2;
        res.success = true;
        return true;
    }

} // namespace mrover
