#include <utility>

#include "cost_map.hpp"

namespace mrover {

    // struct LowResIterator {
    //     using iterator_category = std::forward_iterator_tag;
    //     using difference_type = std::ptrdiff_t;
    //     using value_type = Point;
    //     using pointer = Point const*;
    //     using reference = Point const&;
    //
    //     pointer mPointer;
    //     Eigen::Vector2i mSize;
    //     int mFactor;
    //
    //     Eigen::Vector2i mIndex;
    //
    //     explicit LowResIterator(pointer ptr, Eigen::Vector2i size, int factor)
    //         : mPointer{ptr}, mSize{std::move(size)}, mFactor{factor} {}
    //
    //     auto advance() -> void {
    //         mIndex.x() += mFactor;
    //         if (mIndex.x() >= mSize.x()) {
    //             mIndex.x() = 0;
    //             mIndex.y() += mFactor;
    //         }
    //     }
    //
    //     auto operator*() const -> reference { return mPointer[mIndex.y() * mSize.x() + mIndex.x()]; }
    //
    //     auto operator->() const -> pointer { return &operator*(); }
    //
    //     auto operator++() -> LowResIterator& {
    //         advance();
    //         return *this;
    //     }
    //
    //     auto operator++(int) -> LowResIterator {
    //         auto copy = *this;
    //         advance();
    //         return copy;
    //     }
    //
    //     friend auto operator+(LowResIterator const& it, difference_type n) -> LowResIterator {
    //         auto copy = it;
    //         for (difference_type i = 0; i < n; ++i) {
    //             copy.advance();
    //         }
    //         return copy;
    //     }
    //
    //     friend auto operator==(LowResIterator const& lhs, LowResIterator const& rhs) -> bool {
    //         return lhs.mIndex == rhs.mIndex;
    //     }
    //
    //     friend auto operator!=(LowResIterator const& lhs, LowResIterator const& rhs) -> bool { return !(lhs == rhs); }
    // };

    auto CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;

        try {
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "map", "zed2i_left_camera_frame");
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            for (std::size_t r = 0; r < msg->width; r += mDownSamplingFactor) {
                auto* rowPoints = points + r * msg->width;
                std::size_t beginDownsampled = 0;
                std::size_t endDownsampled = msg->height / mDownSamplingFactor;
                std::for_each(std::execution::par_unseq, beginDownsampled, endDownsampled, [&](std::size_t c) {
                    auto* point = rowPoints + c * mDownSamplingFactor;
                    Eigen::Vector4d pointInCamera{point->x, point->y, point->z, 1};
                    Eigen::Vector4d normalInCamera{point->normal_x, point->normal_y, point->normal_z, 0};

                    mPointsInMap.row(static_cast<Eigen::Index>(r)) = pointInCamera;
                    mNormalsInMap.row(static_cast<Eigen::Index>(r)) = normalInCamera;
                });
            }

            // TODO(neven): Make sure this still works with manif
            mPointsInMap = cameraToMap.transform() * mPointsInMap;
            mNormalsInMap = cameraToMap.transform() * mNormalsInMap;

            for (Eigen::Index r = 0; r < mNumPoints; r++) {
                double x = mPointsInMap(r, 0);
                double y = mPointsInMap(r, 1);
                Eigen::Vector3d normalInMap = mNormalsInMap.row(r).head<3>();

                if (x >= mGlobalGridMsg.info.origin.position.x && x <= mGlobalGridMsg.info.origin.position.x + mDimension &&
                    y >= mGlobalGridMsg.info.origin.position.y && y <= mGlobalGridMsg.info.origin.position.y + mDimension) {
                    int xIndex = std::floor((x - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                    int yIndex = std::floor((y - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                    auto costMapIndex = mGlobalGridMsg.info.width * yIndex + xIndex;

                    // normal.normalize();
                    // get vertical component of (unit) normal vector
                    double z = normalInMap.z();
                    // small z component indicates largely horizontal normal (surface is vertical)
                    signed char cost = z < mNormalThreshold ? OCCUPIED_COST : FREE_COST;

                    mGlobalGridMsg.data[costMapIndex] = std::max(mGlobalGridMsg.data[costMapIndex], cost);
                }
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
            // TODO(neven): Catch TF exceptions only, and log warn them
        }
    }

} // namespace mrover