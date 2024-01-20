#pragma once

#include <chrono>
#include <numeric>
#include <optional>
#include <thread>
#include <unordered_map>

#include <ros/console.h>

/**
 * @brief Profiles the execution time of a loop composed of multiple events.
 */
class LoopProfiler {
    using Clock = std::chrono::high_resolution_clock;
    using EventReadings = std::vector<Clock::duration>;
    using DisplayUnits = std::chrono::milliseconds;

    std::string mName;
    size_t mPrintTick;
    std::unordered_map<std::string, EventReadings> mEventReadings;

    std::optional<Clock::time_point> mLastEpochTime;
    size_t mTick = 0; // Loop iteration counter

public:
    explicit LoopProfiler(std::string_view name, size_t printTick = 120) : mName{name}, mPrintTick{printTick} {}

    /**
     * @brief Call this at the beginning of each loop iteration.
     */
    void beginLoop() {
        if (mTick % mPrintTick == 0) {
            Clock::duration averageLoopDuration{};
            for (auto& [_, durations]: mEventReadings) {
                averageLoopDuration += std::accumulate(durations.begin(), durations.end(), Clock::duration{}) / durations.size();
            }
            // Print update time for the entire loop
            size_t threadId = std::hash<std::thread::id>{}(std::this_thread::get_id());
            auto averageLoopMs = std::chrono::duration_cast<DisplayUnits>(averageLoopDuration);
            int hz = averageLoopMs.count() ? DisplayUnits::period::den / averageLoopMs.count() : -1;
            ROS_DEBUG_STREAM("[" << mName << "] [" << threadId << "] Total: "
                                 << averageLoopMs.count() << "ms"
                                 << " (" << hz << " Hz)");
            // Print update times for each loop event
            for (auto& [name, durations]: mEventReadings) {
                Clock::duration averageEventDuration = std::accumulate(durations.begin(), durations.end(), Clock::duration{}) / durations.size();
                auto averageEventMs = std::chrono::duration_cast<DisplayUnits>(averageEventDuration);
                ROS_DEBUG_STREAM("\t" << name << ": " << averageEventMs.count() << "ms");
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
        Clock::time_point now = Clock::now();
        if (mLastEpochTime) {
            Clock::duration duration = now - mLastEpochTime.value();
            mEventReadings[name].push_back(duration);
        }
        mLastEpochTime = now;
    }
};