#include "Test.h"

// Helper sleep function
void Test::sleepHelper(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Test function movement types, with focus on open loop
void Test::testOpenLoop(Controller* controller) {
    std::vector<float> openLoopSpeeds = {-1.0f, 0.0f, 1.0f, 0.0f};
    const int timePerAction = 500;

    for (auto speed: openLoopSpeeds) {
        for (int i = 0; i < (int) (timePerAction / SLEEP_MS); ++i) {
            controller->moveOpenLoop(speed);
            sleepHelper(SLEEP_MS);
        }
    }
}
