#include <cstdint>
#include <string>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

namespace mrover {

    struct CanNetLink {

        CanNetLink() = default;

        CanNetLink(std::string const& interface, std::uint32_t bitrate, std::uint32_t bitrate_prescaler);

        ~CanNetLink();

        nl_sock* mSocket{};
        rtnl_link* mLink{};
    };

} // namespace mrover
