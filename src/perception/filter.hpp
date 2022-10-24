#pragma once

#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

#include "se3.hpp"

/***
 * A filter that combines multiple readings into one.
 * A user defined proportion acts as a median filter that gets rids of outliers,
 * which are then piped into a mean filter that averages out the values.
 *
 * @tparam T Reading type
 */
template<typename T>
class MeanMedianFilter {
private:
    std::vector<T> mValues;
    std::vector<T> mSortedValues;
    // After sorting, what proportion in the middle values should we use.
    double mProportion;
    // How many readings we have.
    // This will be capped at the capacity, since when we add when full we will overwrite the oldest value.
    size_t mFilterCount = 0;
    // Index to the current head.
    // Note this is a circular buffer, so this will wrap around when we reach the end of the internal vector.
    size_t mHead = 0;

public:
    MeanMedianFilter() : mValues(1), mProportion(0.0) {}

    MeanMedianFilter(size_t size, double centerProportion) : mValues(size), mSortedValues(size), mProportion(centerProportion) {}

    void setFilterCount(size_t filterCount) {
        mValues.resize(filterCount);
    }

    void setProportion(float proportion) {
        mProportion = proportion;
    }

    /**
     * @brief Add a value to the filter, overwrites old values if full.
     */
    void push(T value) {
        mHead = (mHead + 1) % size();
        mValues[mHead] = value;
        mFilterCount = std::min(mFilterCount + 1, size());
        mSortedValues.assign(mValues.begin(), mValues.end());
        std::sort(mSortedValues.begin(), mSortedValues.end());
    }

    void reset() {
        mFilterCount = 0;
    }

    void decrementCount() {
        mFilterCount = std::max(mFilterCount - 1, size_t{});
    }

    [[nodiscard]] size_t size() const {
        return mValues.size();
    }

    [[nodiscard]] size_t filterCount() const {
        return mFilterCount;
    }

    /***
     * @return If we have enough readings to use the filter
     */
    [[nodiscard]] bool ready() const {
        return mFilterCount > 0;
    }

    [[nodiscard]] bool full() const {
        return mFilterCount == size();
    }

    /***
     * @return Filtered reading if full, or else the most recent reading if we don't have enough readings yet.
     */
    [[nodiscard]] T get() const {
        if (!full()) {
            return mValues[mHead];
        }
        auto begin = mSortedValues.begin() + (mProportion * size() / 4);
        auto end = mSortedValues.end() - (mProportion * size() / 4);
        return std::accumulate(begin, end, T{}) / (end - begin);
    }
};

/**
 * @brief Combined filter for XYZ coordinates. Type is always double.
 */
struct XYZFilter {
    MeanMedianFilter<double> fidInOdomX;
    MeanMedianFilter<double> fidInOdomY;
    MeanMedianFilter<double> fidInOdomZ;

    void setFilterParams(size_t count, double proportion);

    void addReading(SE3 const& fidInOdom);

    bool ready() const;

    [[nodiscard]] SE3 getFidInOdom() const;
};