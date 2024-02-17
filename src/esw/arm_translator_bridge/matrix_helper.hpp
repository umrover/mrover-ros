#pragma once

#include <array>

namespace mrover {

    constexpr std::array<std::array<float, 2>, 2> inverseMatrix(std::array<std::array<float, 2>, 2> const& matrix) {
        float determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        float invDet = 1.0f / determinant;

        std::array<std::array<float, 2>, 2> result = {{{matrix[1][1] * invDet, -matrix[0][1] * invDet},
                                                       {-matrix[1][0] * invDet, matrix[0][0] * invDet}}};

        return result;
    }

    static std::array<float, 2> multiplyMatrixByVector(std::array<float, 2> const& vector, std::array<std::array<float, 2>, 2> const& matrix) {
        std::array<float, 2> result = {
                matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
                matrix[1][0] * vector[0] + matrix[1][1] * vector[1]};
        return result;
    }

} // namespace mrover
