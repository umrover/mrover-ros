#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

namespace mrover {

    Eigen::Matrix2f static const CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX{{40, 40},
                                                                     {40, -40}};

    Eigen::Matrix2f static const CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX = CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX.inverse();

    [[maybe_unused]] static auto transformMotorOutputsToPitchRoll(Eigen::Vector2f const& motor01) -> Eigen::Vector2f {
        return CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX * motor01;
    }

    // Function to transform coordinates
    [[maybe_unused]] static auto transformPitchRollToMotorOutputs(Eigen::Vector2f const& pitchRoll) -> Eigen::Vector2f {
        return CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX * pitchRoll;
    }

} // namespace mrover
