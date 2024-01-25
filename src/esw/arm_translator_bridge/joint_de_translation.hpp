#pragma once

#include "matrix_helper.hpp"
#include <array>

namespace mrover {

    constexpr std::array<std::array<float, 2>, 2> CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX = {{{{40, 40}},
                                                                                          {{40, -40}}}};

    constexpr std::array<std::array<float, 2>, 2> CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX = inverseMatrix(CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX);

    static std::pair<float, float> transformMotorOutputsToPitchRoll(float motor_0, float motor_1) {

        std::array<float, 2> motorsVector = {motor_0, motor_1};

        // Perform matrix multiplication
        auto [pitch, roll] = multiplyMatrixByVector(motorsVector, CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX);

        return std::make_pair(pitch, roll);
    }

    // Function to transform coordinates
    static std::pair<float, float> transformPitchRollToMotorOutputs(float pitch, float roll) {
        // Create the input vector
        std::array<float, 2> pitchRollVector = {pitch, roll};

        // Perform matrix multiplication
        auto [m1, m2] = multiplyMatrixByVector(pitchRollVector, CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX);

        return std::make_pair(m1, m2);
    }
} // namespace mrover
