#pragma once

#include "units.hpp"

#include <Eigen/Core>

namespace Eigen {

    template<
            typename Rep1, typename Conversion1, typename MeterExp1, typename KilogramExp1, typename SecondExp1, typename RadianExp1, typename AmpereExp1, typename KelvinExp1, typename ByteExp1, typename TickExp1,
            typename Rep2, typename Conversion2, typename MeterExp2, typename KilogramExp2, typename SecondExp2, typename RadianExp2, typename AmpereExp2, typename KelvinExp2, typename ByteExp2, typename TickExp2>
    struct ScalarBinaryOpTraits<
            mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>,
            mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>,
            internal::scalar_product_op<
                    mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>,
                    mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>>> {
        using U1 = mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>;
        using U2 = mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>;
        using ReturnType = mrover::multiply<U1, U2>;
    };

    template<>
    struct ScalarBinaryOpTraits<
            mrover::Dimensionless,
            mrover::Dimensionless,
            internal::scalar_product_op<mrover::Dimensionless>> {
        using ReturnType = mrover::Dimensionless;
    };

} // namespace Eigen
