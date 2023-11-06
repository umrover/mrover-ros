#include "can_net_link.hpp"

#include <format>
#include <stdexcept>

#include <linux/can.h>
#include <net/if.h>

#include <netlink/route/link.h>

#include <ros/init.h>

namespace mrover {

    CanNetLink::CanNetLink(std::string interface, std::uint32_t bitrate, std::uint32_t bitrate_prescaler)
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

            bool is_up = rtnl_link_get_flags(link) & IFF_UP;

            if (mInterface.starts_with("can")) {
                can_bittiming bt{
                        .bitrate = bitrate,
                        .brp = bitrate_prescaler,
                };
                if (int result = rtnl_link_can_set_bittiming(link, &bt); result < 0) {
                    throw std::runtime_error("Failed to set CAN link bit timings");
                }

                rtnl_link* link_request = rtnl_link_alloc();

                // Trying to send to the socket without this will return error code 22 (invalid argument)
                // By default the MTU is only set for regular CAN frames which are much smaller
                // The effects of these calls will not be realized until "rtnl_link_change"
                rtnl_link_set_mtu(link_request, sizeof(canfd_frame));
                if (is_up) {
                    ROS_WARN("CAN link is already up");
                } else {
                    rtnl_link_set_flags(link_request, IFF_UP);
                }

                if (int result = rtnl_link_change(mSocket, link, link_request, 0); result < 0) {
                    throw std::runtime_error(std::format("Failed to change CAN link: {}", result));
                }
            } else if (mInterface.starts_with("vcan")) {
                if (!is_up) {
                    throw std::runtime_error("Virtual CAN link must be set up manually");
                }
            }

            ROS_INFO_STREAM("Set link up");

        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM("Exception in link setup: " << exception.what());
            ros::shutdown();
        }
    }

    CanNetLink::~CanNetLink() {
        if (!mSocket || !mCache) return;

        rtnl_link* link = rtnl_link_get_by_name(mCache, mInterface.c_str());

        rtnl_link* link_request = rtnl_link_alloc();

        rtnl_link_unset_flags(link_request, IFF_UP);
        if (int result = rtnl_link_change(mSocket, link, link_request, 0); result < 0 && result) {
            std::cerr << std::format("Failed to change CAN link: {}", result) << std::endl;
        }

        nl_socket_free(mSocket);
        mCache = nullptr;
        mSocket = nullptr;
    }

} // namespace mrover
