#include <cstdint>
#include <string>
#include <utility>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

namespace mrover {

    struct CanNetLink {

        CanNetLink() = default;

        CanNetLink(std::string const& interface, std::uint32_t bitrate, std::uint32_t bitrate_prescaler);

        CanNetLink(CanNetLink const&) = delete;
        CanNetLink& operator=(CanNetLink const&) = delete;

        CanNetLink(CanNetLink&& other) noexcept {
            *this = std::move(other);
        }

        CanNetLink& operator=(CanNetLink&& other) noexcept {
            mSocket = std::exchange(other.mSocket, nullptr);
            mLink = std::exchange(other.mLink, nullptr);
            return *this;
        }

        ~CanNetLink();

        nl_sock* mSocket{};
        rtnl_link* mLink{};
    };

} // namespace mrover
