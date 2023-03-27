#pragma once

#include <array>
#include <chrono>
#include <numeric>
#include <string_view>
#include <thread>
#include <unordered_map>

#include <ros/console.h>

using hr_clock = std::chrono::high_resolution_clock;

struct Epoch {
    std::vector<hr_clock::duration> durationSamples;
};

class TimeProfiler {
private:
    size_t mPrintTick;
    std::unordered_map<std::string, Epoch> mEpochDurations;

    hr_clock::time_point mLastEpochTime;
    size_t mTick = 0;

public:
    TimeProfiler(size_t printTick = 60) : mPrintTick{printTick}, mLastEpochTime{hr_clock::now()} {}

    void reset() {
        if (mTick % mPrintTick == 0) return;

        hr_clock::duration total{};
        for (auto& [_, epoch]: mEpochDurations) {
            total += std::accumulate(epoch.durationSamples.begin(), epoch.durationSamples.end(), hr_clock::duration{}) / epoch.durationSamples.size();
        }
        ROS_INFO_STREAM("[" << std::hash<std::thread::id>{}(std::this_thread::get_id()) << "] Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(total).count() << "ms");
        for (auto& [name, epoch]: mEpochDurations) {
            hr_clock::duration epochAverageDuration = std::accumulate(epoch.durationSamples.begin(), epoch.durationSamples.end(), hr_clock::duration{}) / epoch.durationSamples.size();
            ROS_INFO_STREAM("\t" + name + ": " << std::chrono::duration_cast<std::chrono::milliseconds>(epochAverageDuration).count() << "ms");
            epoch.durationSamples.clear();
        }

        mTick++;
    }

    void addEpoch(std::string const& name) {
        hr_clock::time_point now = hr_clock::now();
        hr_clock::duration duration = now - mLastEpochTime;
        mEpochDurations[name].durationSamples.push_back(duration);
        mLastEpochTime = now;
    }
};