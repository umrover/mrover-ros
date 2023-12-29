#include "can_net_link.hpp"

namespace mrover {

    // TODO(quintin): Either remove bitrate settings or make it work
    CanNetLink::CanNetLink(std::string interface, [[maybe_unused]] std::uint32_t bitrate, [[maybe_unused]] std::uint32_t bitrate_prescaler)
        : mInterface{std::move(interface)} {

        try {
            mSocket = nl_socket_alloc();
            if (mSocket == nullptr) {
                throw std::runtime_error("Failed to allocate the network link socket");
            }

            if (int status = nl_connect(mSocket, NETLINK_ROUTE); status < 0) {
                throw std::runtime_error("Failed to connect to the network link socket");
            }

            rtnl_link_alloc_cache(mSocket, AF_UNSPEC, &mCache);
            if (mCache == nullptr) {
                throw std::runtime_error("Failed to allocate the network link cache");
            }

            rtnl_link* link = rtnl_link_get_by_name(mCache, mInterface.c_str());
            if (link == nullptr) {
                throw std::runtime_error(std::format("Failed to retrieve the link {} by name", mInterface));
            }

            if (bool is_up = rtnl_link_get_flags(link) & IFF_UP; !is_up) {
                throw std::runtime_error("CAN link must be set up manually");
            }

            // TODO: literally none of this works bruh. for now just assume it is already up
            // if (mInterface.starts_with("can")) {
            //     // can_bittiming bt{
            //     //         .bitrate = bitrate,
            //     //         .brp = bitrate_prescaler,
            //     // };
            //     // if (int result = rtnl_link_can_set_bittiming(link, &bt); result < 0) {
            //     //     throw std::runtime_error("Failed to set CAN link bit timings");
            //     // }
            //
            //     // See https://android.googlesource.com/platform/external/libnl/+/bbcb553f0f4636dd40e84d320a576b3de7b95357/lib/route/link.c
            //     // Specifically the section "@par 2) Changing Link attributes"
            //     rtnl_link* link_request = rtnl_link_alloc();
            //
            //     // Trying to send to the socket without this will return error code 22 (invalid argument)
            //     // By default the MTU is configured for regular CAN frame which are much smaller
            //     // The effects of these calls will not be realized until "rtnl_link_change"
            //     // rtnl_link_set_mtu(link_request, sizeof(canfd_frame));
            //     if (is_up) {
            //         ROS_WARN("CAN link is already up");
            //     } else {
            //         rtnl_link_set_flags(link_request, IFF_UP);
            //     }
            //
            //     if (int result = rtnl_link_change(mSocket, link, link_request, 0); result < 0) {
            //         rtnl_link_put(link); // TODO(quintin) Use RAII here
            //         throw std::runtime_error(std::format("Failed to change CAN link: {}", result));
            //     }
            //
            //     rtnl_link_put(link); // Free
            //
            // } else if (mInterface.starts_with("vcan")) {
            //     if (!is_up) {
            //         throw std::runtime_error("Virtual CAN link must be set up manually");
            //     }
            // }

            ROS_INFO_STREAM("Set link up");

        } catch (std::exception const& exception) {
            ROS_FATAL_STREAM("Exception in link setup: " << exception.what());
            ros::shutdown();
        }
    }

    CanNetLink::~CanNetLink() {
        if (!mSocket || !mCache) return;

        // TODO: just assume interface is always up
        // if (mInterface.starts_with("can")) {
        //     rtnl_link *link = rtnl_link_get_by_name(mCache, mInterface.c_str()), *link_request = rtnl_link_alloc();
        //
        //     rtnl_link_unset_flags(link_request, IFF_UP);
        //     if (int result = rtnl_link_change(mSocket, link, link_request, 0); result < 0) {
        //         std::cerr << std::format("Failed to change CAN link: {}", result) << std::endl;
        //     }
        //
        //     rtnl_link_put(link); // Free
        // }

        nl_socket_free(mSocket);
        mCache = nullptr;
        mSocket = nullptr;
    }

} // namespace mrover
