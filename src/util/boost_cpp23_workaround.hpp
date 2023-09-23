#pragma once

#include <memory>
#include <type_traits>

// Quintin: This is heinous, but it's the only way I can get the whole code to compile with anything greater than C++17
#ifdef __GNUC__
#define GENERATE_ALLOCATOR_SPECIALIZATION(_Tp)                                               \
    namespace std {                                                                          \
                                                                                             \
        template<>                                                                           \
        class allocator<_Tp> : public __allocator_base<_Tp> {                                \
        public:                                                                              \
            typedef _Tp value_type;                                                          \
            typedef size_t size_type;                                                        \
            typedef ptrdiff_t difference_type;                                               \
                                                                                             \
            typedef _Tp* pointer;                                                            \
            typedef const _Tp* const_pointer;                                                \
            typedef _Tp& reference;                                                          \
            typedef const _Tp& const_reference;                                              \
                                                                                             \
            template<typename _Tp1>                                                          \
            struct rebind {                                                                  \
                typedef allocator<_Tp1> other;                                               \
            };                                                                               \
                                                                                             \
            using propagate_on_container_move_assignment = true_type;                        \
                                                                                             \
            using is_always_equal = true_type;                                               \
                                                                                             \
            __attribute__((__always_inline__))                                               \
            _GLIBCXX20_CONSTEXPR                                                             \
            allocator() _GLIBCXX_NOTHROW {}                                                  \
                                                                                             \
            __attribute__((__always_inline__))                                               \
            _GLIBCXX20_CONSTEXPR                                                             \
            allocator(const allocator& __a) _GLIBCXX_NOTHROW                                 \
                : __allocator_base<_Tp>(__a) {}                                              \
                                                                                             \
            allocator& operator=(const allocator&) = default;                                \
                                                                                             \
            template<typename _Tp1>                                                          \
            __attribute__((__always_inline__))                                               \
            _GLIBCXX20_CONSTEXPR                                                             \
            allocator(const allocator<_Tp1>&) _GLIBCXX_NOTHROW {}                            \
                                                                                             \
            __attribute__((__always_inline__)) ~allocator() _GLIBCXX_NOTHROW {}              \
                                                                                             \
            [[nodiscard, __gnu__::__always_inline__]] _Tp*                                   \
            allocate(size_t __n) {                                                           \
                return __allocator_base<_Tp>::allocate(__n, 0);                              \
            }                                                                                \
                                                                                             \
            [[__gnu__::__always_inline__]] void                                              \
            deallocate(_Tp* __p, size_t __n) {                                               \
                __allocator_base<_Tp>::deallocate(__p, __n);                                 \
            }                                                                                \
                                                                                             \
            friend __attribute__((__always_inline__)) _GLIBCXX20_CONSTEXPR bool              \
            operator==(const allocator&, const allocator&) _GLIBCXX_NOTHROW { return true; } \
        };                                                                                   \
    }

#include <boost/shared_ptr.hpp>
using _TpGenerated1 = boost::shared_ptr<void>;
GENERATE_ALLOCATOR_SPECIALIZATION(_TpGenerated1)

#include <boost/signals2/detail/foreign_ptr.hpp>
#include <boost/variant.hpp>
using _TpGenerated2 = boost::variant<boost::shared_ptr<void>, boost::signals2::detail::foreign_void_shared_ptr>;
GENERATE_ALLOCATOR_SPECIALIZATION(_TpGenerated2)

#endif
