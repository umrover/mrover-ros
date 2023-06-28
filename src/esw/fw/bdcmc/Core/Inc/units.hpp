#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <ratio>
#include <type_traits>

namespace mrover {

    //
    //  Concepts
    //

    template<typename T>
    concept Ratioable = requires(T t) {
        { t.num } -> std::convertible_to<std::intmax_t>;
        { t.den } -> std::convertible_to<std::intmax_t>;
    };

    template<typename T>
    concept Unitable = requires(T t) {
        typename T::conversion_t;
        typename T::meter_exp_t;
        typename T::kilogram_exp_t;
        typename T::second_exp_t;
        typename T::radian_exp_t;
        typename T::ampere_exp_t;
        typename T::kelvin_exp_t;
        typename T::byte_exp_t;
        requires Ratioable<typename T::conversion_t>;
        requires Ratioable<typename T::meter_exp_t>;
        requires Ratioable<typename T::kilogram_exp_t>;
        requires Ratioable<typename T::second_exp_t>;
        requires Ratioable<typename T::radian_exp_t>;
        requires Ratioable<typename T::ampere_exp_t>;
        requires Ratioable<typename T::kelvin_exp_t>;
        requires Ratioable<typename T::byte_exp_t>;
        { t.get() } -> std::convertible_to<double>;
    };

    template<typename T>
    concept Scalar = std::convertible_to<T, double>;

    template<typename U1, typename U2>
    concept Convertible =
            Unitable<U1> && Unitable<U2> &&
            std::is_same_v<typename U1::meter_exp_t, typename U2::meter_exp_t> &&
            std::is_same_v<typename U1::kilogram_exp_t, typename U2::kilogram_exp_t> &&
            std::is_same_v<typename U1::second_exp_t, typename U2::second_exp_t> &&
            std::is_same_v<typename U1::radian_exp_t, typename U2::radian_exp_t> &&
            std::is_same_v<typename U1::ampere_exp_t, typename U2::ampere_exp_t> &&
            std::is_same_v<typename U1::kelvin_exp_t, typename U2::kelvin_exp_t> &&
            std::is_same_v<typename U1::byte_exp_t, typename U2::byte_exp_t>;

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
    template<Ratioable Conversion = std::ratio<1>,
             Ratioable MeterExp = zero_exp_t,
             Ratioable KilogramExp = zero_exp_t,
             Ratioable SecondExp = zero_exp_t,
             Ratioable RadianExp = zero_exp_t,
             Ratioable AmpereExp = zero_exp_t,
             Ratioable KelvinExp = zero_exp_t,
             Ratioable ByteExp = zero_exp_t>
        requires(Conversion::num != 0 && Conversion::den != 0)
    struct Unit {
        using conversion_t = Conversion;
        using meter_exp_t = MeterExp;
        using kilogram_exp_t = KilogramExp;
        using second_exp_t = SecondExp;
        using radian_exp_t = RadianExp;
        using ampere_exp_t = AmpereExp;
        using kelvin_exp_t = KelvinExp;
        using byte_exp_t = ByteExp;

        constexpr static double CONVERSION = static_cast<double>(conversion_t::num) / conversion_t::den;

        // Stores the SI base unit value, NOT the transformed unit value based on the conversion ratio
        double rep{};

        [[nodiscard]] constexpr auto get() const -> double {
            return rep / CONVERSION;
        }

        constexpr Unit() = default;

        constexpr explicit Unit(double d) : rep{d * CONVERSION} {}

        template<Unitable U>
        constexpr Unit(U const& other)
            requires Convertible<Unit, U>
        {
            rep = other.get();
        }

        template<Unitable U>
        [[nodiscard]] constexpr auto operator=(U const& rhs) -> Unit&
            requires Convertible<Unit, U>
        {
            rep = rhs.get();
            return *this;
        }
    };

    template<Unitable U>
    inline constexpr auto make_unit(Scalar auto const& value) -> U {
        return U{static_cast<double>(value) * U::CONVERSION};
    }

    using dimensionless_t = Unit<>;

    //
    //  Static Operations
    //

    namespace detail {

        template<Unitable U>
        struct inverse {
            using inverse_ratio_t = std::ratio<-1>;
            using type = Unit<std::ratio<U::conversion_t::den, U::conversion_t::num>,
                              std::ratio_multiply<typename U::meter_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::kilogram_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::second_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::radian_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::ampere_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::kelvin_exp_t, inverse_ratio_t>,
                              std::ratio_multiply<typename U::byte_exp_t, inverse_ratio_t>>;
        };

        template<Unitable U1, Unitable U2>
        struct unit_multiply {
            using type = Unit<std::ratio_multiply<typename U1::conversion_t, typename U2::conversion_t>,
                              std::ratio_add<typename U1::meter_exp_t, typename U2::meter_exp_t>,
                              std::ratio_add<typename U1::kilogram_exp_t, typename U2::kilogram_exp_t>,
                              std::ratio_add<typename U1::second_exp_t, typename U2::second_exp_t>,
                              std::ratio_add<typename U1::radian_exp_t, typename U2::radian_exp_t>,
                              std::ratio_add<typename U1::ampere_exp_t, typename U2::ampere_exp_t>,
                              std::ratio_add<typename U1::kelvin_exp_t, typename U2::kelvin_exp_t>,
                              std::ratio_add<typename U1::byte_exp_t, typename U2::byte_exp_t>>;
        };

        template<Unitable U, Unitable... Us>
        struct compound;

        template<Unitable U>
        struct compound<U> {
            using type = U;
        };

        template<Unitable U1, Unitable U2, Unitable... Us>
        struct compound<U1, U2, Us...> {
            using type = typename compound<typename unit_multiply<U1, U2>::type, Us...>::type;
        };

    } // namespace detail

    template<Unitable U1, Unitable U2>
    using multiply = typename detail::unit_multiply<U1, U2>::type;

    template<Unitable U>
    using inverse = typename detail::inverse<U>::type;

    template<Unitable... Us>
    using compound_unit = typename detail::compound<Us...>::type;

    //
    //  Runtime Operations
    //

    template<Unitable U>
    inline constexpr auto operator*(Scalar auto const& n, U const& u) {
        return U{static_cast<double>(n) * u.rep};
    }

    template<Unitable U>
    inline constexpr auto operator*(U const& u, Scalar auto const& n) {
        return U{u.rep * static_cast<double>(n)};
    }

    template<Unitable U1, Unitable U2>
    inline constexpr auto operator*(U1 const& u1, U2 const& u2) {
        return multiply<U1, U2>{u1.rep * u2.rep};
    }

    inline constexpr auto operator*=(Unitable auto& u, Scalar auto const& n) {
        return u = u * static_cast<double>(n);
    }

    template<Unitable U>
    inline constexpr auto operator/(U const& u, Scalar auto const& n) {
        return U{u.rep / static_cast<double>(n)};
    }

    template<Unitable U>
    inline constexpr auto operator/(Scalar auto const& n, U const& u) {
        return inverse<U>{static_cast<double>(n) / u.rep};
    }

    template<Unitable U1, Unitable U2>
    inline constexpr auto operator/(U1 const& u1, U2 const& u2) {
        return multiply<U1, inverse<U2>>{u1.rep / u2.rep};
    }

    inline constexpr auto operator/=(Unitable auto& u, Scalar auto const& n) {
        return u = u / static_cast<double>(n);
    }

    template<Unitable U>
    inline constexpr auto operator-(U const& u) {
        return U{-u.rep};
    }

    template<Unitable U>
    inline constexpr auto operator+(U const& u1, Unitable auto const& u2) {
        return U{u1.rep + u2.rep};
    }

    template<Unitable U>
    inline constexpr auto operator-(U const& u1, Unitable auto const& u2) {
        return U{u1.rep - u2.rep};
    }

    inline constexpr auto operator+=(Unitable auto& u1, Unitable auto const& u2) {
        return u1 = u1 + u2;
    }

    inline constexpr auto operator-=(Unitable auto& u1, Unitable auto const& u2) {
        return u1 = u1 - u2;
    }

    inline constexpr auto operator<=>(Unitable auto const& u1, Unitable auto const& u2) {
        return u1.rep <=> u2.rep;
    }

    template<Unitable U>
    inline constexpr auto abs(U const& u) {
        return U{std::fabs(u.rep)};
    }

    template<Unitable U>
    inline constexpr auto clamp(U const& u, Unitable auto const& min, Unitable auto const& max) {
        return U{std::clamp(u.rep, min.rep, max.rep)};
    }

    //
    //  Common units
    //

    // First template is conversion ratio to base SI unit
    // Following template arguments are exponents in the order: Meter, Kilogram, Second, Radian, Ampere, Kelvin, Byte
    using Meters = Unit<std::ratio<1>, std::ratio<1>>;
    using Centimeters = Unit<std::centi, std::ratio<1>>;
    using Millimeters = Unit<std::milli, std::ratio<1>>;
    using Seconds = Unit<std::ratio<1>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Milliseconds = Unit<std::milli, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Microseconds = Unit<std::micro, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Nanoseconds = Unit<std::nano, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Minutes = Unit<std::ratio<60>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Hours = Unit<std::ratio<3600>, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Hertz = inverse<Seconds>;
    using Radians = Unit<std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using RadiansPerSecond = compound_unit<Radians, inverse<Seconds>>;
    using Amperes = Unit<std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Kelvin = Unit<std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Bytes = Unit<std::ratio<1>, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, zero_exp_t, std::ratio<1>>;
    using Volts = Unit<std::ratio<1>, std::ratio<2>, std::ratio<1>, std::ratio<-3>, zero_exp_t, std::ratio<-1>, zero_exp_t>;

} // namespace mrover
