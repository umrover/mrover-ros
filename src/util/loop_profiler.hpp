#pragma once

#include <array>
#include <chrono>
#include <numeric>
#include <thread>
#include <unordered_map>

#include <ros/console.h>

/**
 * @brief Profiles the execution time of a loop composed of multiple events.
 */
class LoopProfiler {
private:
    using hr_clock = std::chrono::high_resolution_clock;
    using EventReadings = std::vector<hr_clock::duration>;

    size_t mPrintTick;
    std::unordered_map<std::string, EventReadings> mEventReadings;

    hr_clock::time_point mLastEpochTime;
    size_t mTick = 0; // loop iteration counter

public:
    LoopProfiler(size_t printTick = 60) : mPrintTick{printTick}, mLastEpochTime{hr_clock::now()} {}

    /**
     * @brief Call this at the end of each loop iteration.
     */
    void finishLoop() {
        if (mTick % mPrintTick == 0) {
            hr_clock::duration averageLoopDuration{};
            for (auto& [_, durations]: mEventReadings) {
                averageLoopDuration += std::accumulate(durations.begin(), durations.end(), hr_clock::duration{}) / durations.size();
            }
            // Print update time for the entire loop
            size_t threadId = std::hash<std::thread::id>{}(std::this_thread::get_id());
            auto averageLoopMs = std::chrono::duration_cast<std::chrono::milliseconds>(averageLoopDuration);
            ROS_INFO_STREAM("[" << threadId << "] Total: " << averageLoopMs.count() << "ms");
            // Print update times for each loop event
            for (auto& [name, durations]: mEventReadings) {
                hr_clock::duration averageEventDuration = std::accumulate(durations.begin(), durations.end(), hr_clock::duration{}) / durations.size();
                auto averageEventMs = std::chrono::duration_cast<std::chrono::milliseconds>(averageEventDuration);
                ROS_INFO_STREAM("\t" + name + ": " << averageEventMs.count() << "ms");
                durations.clear();
            }
        }

        mTick++;
    }

    /**
     * @brief Call this at the beginning of each event in the loop.
     *
     * This signals the end of the previous event and starts a new one.
     *
     * @param name
     */
    void measureEvent(std::string const& name) {
        hr_clock::time_point now = hr_clock::now();
        hr_clock::duration duration = now - mLastEpochTime;
        mEventReadings[name].push_back(duration);
        mLastEpochTime = now;
    }
};