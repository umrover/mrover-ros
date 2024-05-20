#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <numbers>
#include <ratio>
#include <type_traits>

namespace mrover {

    //
    //  Concepts
    //

    template<typename T>
    concept IsRatio = requires(T) {
        { T::num } -> std::convertible_to<std::intmax_t>;
        { T::den } -> std::convertible_to<std::intmax_t>;
    };

    template<typename T>
    concept IsArithmetic = std::is_arithmetic_v<T>;

    template<typename T>
    concept IsUnit = requires(T t) {
        typename T::conversion_t;
        typename T::meter_exp_t;
        typename T::kilogram_exp_t;
        typename T::second_exp_t;
        typename T::radian_exp_t;
        typename T::ampere_exp_t;
        typename T::kelvin_exp_t;
        typename T::byte_exp_t;
        typename T::tick_exp_t;
        requires IsRatio<typename T::conversion_t>;
        requires IsRatio<typename T::meter_exp_t>;
        requires IsRatio<typename T::kilogram_exp_t>;
        requires IsRatio<typename T::second_exp_t>;
        requires IsRatio<typename T::radian_exp_t>;
        requires IsRatio<typename T::ampere_exp_t>;
        requires IsRatio<typename T::kelvin_exp_t>;
        requires IsRatio<typename T::byte_exp_t>;
        requires IsRatio<typename T::tick_exp_t>;
        { t.get() } -> std::convertible_to<typename T::rep_t>;
    };

    template<typename U1, typename U2>
    concept AreExponentsSame =
            IsUnit<U1> && IsUnit<U2> &&
            std::is_same_v<typename U1::meter_exp_t, typename U2::meter_exp_t> &&
            std::is_same_v<typename U1::kilogram_exp_t, typename U2::kilogram_exp_t> &&
            std::is_same_v<typename U1::second_exp_t, typename U2::second_exp_t> &&
            std::is_same_v<typename U1::radian_exp_t, typename U2::radian_exp_t> &&
            std::is_same_v<typename U1::ampere_exp_t, typename U2::ampere_exp_t> &&
            std::is_same_v<typename U1::kelvin_exp_t, typename U2::kelvin_exp_t> &&
            std::is_same_v<typename U1::byte_exp_t, typename U2::byte_exp_t> &&
            std::is_same_v<typename U1::tick_exp_t, typename U2::tick_exp_t>;

    //
    //  Structs
    //

    using zero_exp_t = std::ratio<0>;

    /**
     * @brief International System of Units (SI) base units
     *
     * @tparam Conversion   Conversion ratio to base SI unit. For example, for
     * millimeters, this would be std::milli (1/1000) All other template
     * parameters are exponents of the base SI units.
     */
    template<IsArithmetic Rep,
             IsRatio Conversion = std::ratio<1>,
             IsRatio MeterExp = zero_exp_t,
             IsRatio KilogramExp = zero_exp_t,
             IsRatio SecondExp = zero_exp_t,
             IsRatio RadianExp = zero_exp_t,
             IsRatio AmpereExp = zero_exp_t,
             IsRatio KelvinExp = zero_exp_t,
             IsRatio ByteExp = zero_exp_t,
             IsRatio TickExp = zero_exp_t>
        requires(Conversion::num != 0 && Conversion::den != 0)
    struct Unit {
        using rep_t = Rep;
        using conversion_t = Conversion;
        using meter_exp_t = MeterExp;
        using kilogram_exp_t = KilogramExp;
        using second_exp_t = SecondExp;
        using radian_exp_t = RadianExp;
        using ampere_exp_t = AmpereExp;
        using kelvin_exp_t = KelvinExp;
        using byte_exp_t = ByteExp;
        using tick_exp_t = TickExp;

        constexpr static Rep CONVERSION = static_cast<Rep>(conversion_t::num) / conversion_t::den;

        // Stores the SI base unit value, NOT the transformed unit value based on the conversion ratio
        Rep rep{};

        [[nodiscard]] constexpr auto get() const -> Rep {
            return rep / CONVERSION;
        }

        constexpr Unit() = default;
        constexpr explicit Unit(IsArithmetic auto const& val) : rep{static_cast<Rep>(val) * CONVERSION} {}

        template<IsUnit U>
        constexpr Unit(U const& other)
            requires AreExponentsSame<Unit, U>
        {
            rep = other.rep;
        }

        template<IsUnit U>
        constexpr auto operator=(U const& rhs) -> Unit&
            requires AreExponentsSame<Unit, U>
        {
            rep = rhs.rep;
            return *this;
        }
    };

    template<>
    struct Unit<float> {
        using rep_t = float;
        using conversion_t = std::ratio<1>;
        using meter_exp_t = zero_exp_t;
        using kilogram_exp_t = zero_exp_t;
        using second_exp_t = zero_exp_t;
        using radian_exp_t = zero_exp_t;
        using ampere_exp_t = zero_exp_t;
        using kelvin_exp_t = zero_exp_t;
        using byte_exp_t = zero_exp_t;
        using tick_exp_t = zero_exp_t;

        constexpr static rep_t CONVERSION = 1.0f;

        // Stores the SI base unit value, NOT the transformed unit value based on the conversion ratio
        rep_t rep{};

        [[nodiscard]] constexpr auto get() const -> float {
            return rep;
        }

        constexpr Unit() = default;
        constexpr Unit(IsArithmetic auto const& val) : rep{static_cast<rep_t>(val)} {}

        template<IsUnit U>
        constexpr Unit(U const& other)
            requires AreExponentsSame<Unit, U>
            : rep{other.rep} {}

        template<IsUnit U>
        [[nodiscard]] constexpr auto operator=(U const& rhs) -> Unit&
            requires AreExponentsSame<Unit, U>
        {
            rep = rhs.rep;
            return *this;
        }

        constexpr auto operator=(IsArithmetic auto const& rhs) -> Unit& {
            rep = static_cast<rep_t>(rhs);
            return *this;
        }
    };

    //
    //  Static Operations
    //

    template<IsUnit U>
    static auto constexpr make_no_conversion(IsArithmetic auto const& other) -> U {
        U result;
        result.rep = static_cast<typename U::rep_t>(other);
        return result;
    }

    namespace detail {

        template<IsUnit U, IsRatio Ratio>
        struct exp_multiply {
            using type = Unit<typename U::rep_t,
                              std::ratio<U::conversion_t::den, U::conversion_t::num>,
                              std::ratio_multiply<typename U::meter_exp_t, Ratio>,
                              std::ratio_multiply<typename U::kilogram_exp_t, Ratio>,
                              std::ratio_multiply<typename U::second_exp_t, Ratio>,
                              std::ratio_multiply<typename U::radian_exp_t, Ratio>,
                              std::ratio_multiply<typename U::ampere_exp_t, Ratio>,
                              std::ratio_multiply<typename U::kelvin_exp_t, Ratio>,
                              std::ratio_multiply<typename U::byte_exp_t, Ratio>,
                              std::ratio_multiply<typename U::tick_exp_t, Ratio>>;
        };

        template<IsUnit U1, IsUnit U2>
        struct exp_add {
            using type = Unit<decltype(typename U1::rep_t{} * typename U2::rep_t{}),
                              std::ratio_multiply<typename U1::conversion_t, typename U2::conversion_t>,
                              std::ratio_add<typename U1::meter_exp_t, typename U2::meter_exp_t>,
                              std::ratio_add<typename U1::kilogram_exp_t, typename U2::kilogram_exp_t>,
                              std::ratio_add<typename U1::second_exp_t, typename U2::second_exp_t>,
                              std::ratio_add<typename U1::radian_exp_t, typename U2::radian_exp_t>,
                              std::ratio_add<typename U1::ampere_exp_t, typename U2::ampere_exp_t>,
                              std::ratio_add<typename U1::kelvin_exp_t, typename U2::kelvin_exp_t>,
                              std::ratio_add<typename U1::byte_exp_t, typename U2::byte_exp_t>,
                              std::ratio_add<typename U1::tick_exp_t, typename U2::tick_exp_t>>;
        };

        template<IsUnit U, IsUnit... Us>
        struct compound;

        template<IsUnit U>
        struct compound<U> {
            using type = U;
        };

        template<IsUnit U1, IsUnit U2, IsUnit... Us>
        struct compound<U1, U2, Us...> {
            using type = typename compound<typename exp_add<U1, U2>::type, Us...>::type;
        };

        template<IsUnit U>
        struct strip_conversion {
            using type = Unit<typename U::rep_t,
                              std::ratio<1>,
                              typename U::meter_exp_t,
                              typename U::kilogram_exp_t,
                              typename U::second_exp_t,
                              typename U::radian_exp_t,
                              typename U::ampere_exp_t,
                              typename U::kelvin_exp_t,
                              typename U::byte_exp_t,
                              typename U::tick_exp_t>;
        };

    } // namespace detail

    template<IsUnit U1, IsUnit U2>
    using multiply = typename detail::exp_add<U1, U2>::type;

    template<IsUnit U>
    using inverse = typename detail::exp_multiply<U, std::ratio<-1>>::type;

    template<IsUnit U>
    using root = typename detail::exp_multiply<U, std::ratio<1, 2>>::type;

    template<IsUnit... Us>
    using compound_unit = typename detail::compound<Us...>::type;

    //
    //  Runtime Operations
    //

    template<IsUnit U>
    constexpr auto operator*(IsArithmetic auto const& n, U const& u) {
        return make_no_conversion<U>(n * u.rep);
    }

    template<IsUnit U>
    constexpr auto operator*(U const& u, IsArithmetic auto const& n) {
        return make_no_conversion<U>(u.rep * n);
    }

    template<IsUnit U1, IsUnit U2>
    constexpr auto operator*(U1 const& u1, U2 const& u2) {
        return make_no_conversion<multiply<U1, U2>>(u1.rep * u2.rep);
    }

    template<IsUnit U>
    constexpr auto operator*=(U& u, IsArithmetic auto const& n) {
        return u = u * n;
    }

    template<IsUnit U>
    constexpr auto operator/(U const& u, IsArithmetic auto const& n) {
        return make_no_conversion<U>(u.rep / n);
    }

    template<IsUnit U>
    constexpr auto operator/(IsArithmetic auto const& n, U const& u) {
        return make_no_conversion<inverse<U>>(n / u.rep);
    }

    template<IsUnit U1, IsUnit U2>
    constexpr auto operator/(U1 const& u1, U2 const& u2) {
        return make_no_conversion<multiply<U1, inverse<U2>>>(u1.rep / u2.rep);
    }

    template<IsUnit U>
    constexpr auto operator/=(U& u, IsArithmetic auto const& n) {
        return u = u / n;
    }

    template<IsUnit U>
    constexpr auto operator-(U const& u) {
        return make_no_conversion<U>(-u.rep);
    }

    template<IsUnit U1, IsUnit U2>
        requires AreExponentsSame<U1, U2>
    constexpr auto operator+(U1 const& u1, U2 const& u2) {
        return make_no_conversion<U1>(u1.rep + u2.rep);
    }

    template<IsUnit U1, IsUnit U2>
        requires AreExponentsSame<U1, U2>
    constexpr auto operator-(U1 const& u1, U2 const& u2) {
        return make_no_conversion<U1>(u1.rep - u2.rep);
    }

    constexpr auto operator+=(IsUnit auto& u1, IsUnit auto const& u2) {
        return u1 = u1 + u2;
    }

    constexpr auto operator-=(IsUnit auto& u1, IsUnit auto const& u2) {
        return u1 = u1 - u2;
    }

    constexpr auto operator<=>(IsUnit auto const& u1, IsUnit auto const& u2) {
        return u1.rep <=> u2.rep;
    }

    template<IsUnit U>
    constexpr auto sqrt(U const& u) {
        return make_no_conversion<root<U>>(std::sqrt(u.rep));
    }

    constexpr auto square(IsUnit auto const& u) {
        return u * u;
    }

    template<IsUnit U>
    constexpr auto abs(U const& u) {
        return make_no_conversion<U>(std::fabs(u.rep));
    }

    template<IsUnit U>
    constexpr auto signum(U const& u) -> int {
        bool is_zero = std::fabs(u.rep) < std::numeric_limits<typename U::rep_t>::epsilon();
        return is_zero ? 0 : std::signbit(u.rep) ? -1
                                                 : 1;
    }

    template<IsUnit U>
    constexpr auto fmod(U const& u, typename U::rep_t n) {
        return make_no_conversion<U>(std::fmod(u.rep, n));
    }

    //
    //  Common units
    //

    constexpr auto TAU = 2 * std::numbers::pi;

    // First template is conversion ratio to base SI unit
    // Following template arguments are exponents in the order: Meter, Kilogram, Second, Radian, Ampere, Kelvin, Byte
    using default_rep_t = float;
    using Dimensionless = Unit<default_rep_t>;
    using Ratio = Dimensionless;
    using Percent = Dimensionless;
    using Meters = Unit<default_rep_t, std::ratio<1>, std::ratio<1>>;
    using Centimeters = Unit<default_rep_t, std::centi, std::ratio<1>>;
    using Millimeters = Unit<default_rep_t, std::milli, std::ratio<1>>;
    using Seconds = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Milliseconds = Unit<default_rep_t, std::milli, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Microseconds = Unit<default_rep_t, std::micro, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Nanoseconds = Unit<default_rep_t, std::nano, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Minutes = Unit<default_rep_t, std::ratio<60>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Hours = Unit<default_rep_t, std::ratio<3600>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Hertz = inverse<Seconds>;
    using Radians = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using tau_ratio_t = std::ratio<62831853071795864, 10000000000000000>; // TODO(quintin) is this 0-head play?
    using Revolutions = Unit<default_rep_t, tau_ratio_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using RadiansPerSecond = compound_unit<Radians, inverse<Seconds>>;
    using Amperes = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Kelvin = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Bytes = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Volts = Unit<default_rep_t, std::ratio<1>, std::ratio<2>, std::ratio<1>, std::ratio<-3>, zero_exp_t, std::ratio<-1>>;
    using Ticks = Unit<default_rep_t, std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;

    using RevolutionsPerSecond = compound_unit<Revolutions, inverse<Seconds>>;
    using MetersPerSecond = compound_unit<Meters, inverse<Seconds>>;
    using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;

    //
    // Literals
    //

    constexpr auto operator""_m(unsigned long long int n) {
        return Meters{n};
    }

    constexpr auto operator""_cm(unsigned long long int n) {
        return Centimeters{n};
    }

    constexpr auto operator""_mm(unsigned long long int n) {
        return Millimeters{n};
    }

    constexpr auto operator""_Hz(unsigned long long int n) {
        return Hertz{n};
    }

    constexpr auto operator""_rad(unsigned long long int n) {
        return Radians{n};
    }

    constexpr auto operator""_rad_per_s(unsigned long long int n) {
        return RadiansPerSecond{n};
    }

    constexpr auto operator""_A(unsigned long long int n) {
        return Amperes{n};
    }

    constexpr auto operator""_K(unsigned long long int n) {
        return Kelvin{n};
    }

    constexpr auto operator""_B(unsigned long long int n) {
        return Bytes{n};
    }

    constexpr auto operator""_V(unsigned long long int n) {
        return Volts{n};
    }

    constexpr auto operator""_mps(unsigned long long int n) {
        return MetersPerSecond{n};
    }

    constexpr auto operator""_percent(unsigned long long int n) {
        return Percent{n};
    }

    constexpr auto operator""_ticks(unsigned long long int n) {
        return Ticks{n};
    }

    //
    // Specialization
    //

    // constexpr auto operator*(IsArithmetic auto const& n, Radians const& r) {
    //     return Radians{std::fmod(static_cast<typename Radians::rep_t>(n) * r.rep, TAU) + static_cast<typename Radians::rep_t>(n) < 0 ? TAU : 0};
    // }
    //
    // constexpr auto operator*(Radians const& r, IsArithmetic auto const& n) {
    //     return operator*(n, r);
    // }
    //
    // constexpr auto operator-(Radians const& r) {
    //     return Radians{std::fmod(TAU - r.rep, TAU)};
    // }
    //
    // constexpr auto operator+(Radians const& r1, Radians const& r2) {
    //     return Radians{std::fmod(r1.rep + r2.rep, TAU)};
    // }
    //
    // constexpr auto operator-(Radians const& r1, Radians const& r2) {
    //     return Radians{std::fmod(r1.rep - r2.rep + TAU, TAU)};
    // }

} // namespace mrover
