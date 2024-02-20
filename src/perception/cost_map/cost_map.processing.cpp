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
            manif::SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "map", "zed2i_left_camera_frame").cast<float>();
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            for (std::size_t i = 0, r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (std::size_t c = 0; c < msg->width; c += mDownSamplingFactor) {
                    auto* point = points + r * msg->width + c;
                    Eigen::Vector4f pointInCamera{point->x, point->y, point->z, 1};
                    Eigen::Vector4f normalInCamera{point->normal_x, point->normal_y, point->normal_z, 0};

                    mPointsInMap.col(static_cast<Eigen::Index>(i)) = cameraToMap.transform() * pointInCamera;
                    mNormalsInMap.col(static_cast<Eigen::Index>(i)) = cameraToMap.transform() * normalInCamera;
                    i++;
                }
            }

            for (Eigen::Index i = 0; i < mNumPoints; i++) {
                double xInMap = mPointsInMap.col(i).x();
                double yInMap = mPointsInMap.col(i).y();
                Eigen::Vector3f normalInMap = mNormalsInMap.col(i).head<3>();

                if (xInMap >= mGlobalGridMsg.info.origin.position.x && xInMap <= mGlobalGridMsg.info.origin.position.x + mDimension &&
                    yInMap >= mGlobalGridMsg.info.origin.position.y && yInMap <= mGlobalGridMsg.info.origin.position.y + mDimension) {
                    int xIndex = std::floor((xInMap - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution);
                    int yIndex = std::floor((yInMap - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution);
                    auto costMapIndex = mGlobalGridMsg.info.width * yIndex + xIndex;

                    // Z is the vertical component of of hte normal
                    // A small Z component indicates largely horizontal normal (surface is vertical)
                    std::int8_t cost = normalInMap.z() < mNormalThreshold ? OCCUPIED_COST : FREE_COST;

                    mGlobalGridMsg.data[costMapIndex] = std::max(mGlobalGridMsg.data[costMapIndex], cost);
                }
            }
            mCostMapPub.publish(mGlobalGridMsg);
        } catch (...) {
            // TODO(neven): Catch TF exceptions only, and log warn them
        }
    }

} // namespace mrover