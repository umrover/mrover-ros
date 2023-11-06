#include "can_net_link.hpp"

#include <format>
#include <stdexcept>

#include <linux/can.h>
#include <net/if.h>

#include <netlink/route/link.h>

#include <ros/console.h>

namespace mrover {

    CanNetLink::CanNetLink(std::string const& interface, std::uint32_t bitrate, std::uint32_t bitrate_prescaler) {
        try {
            mSocket = nl_socket_alloc();
            if (mSocket == nullptr) {
                throw std::runtime_error("Failed to allocate the network link socket");
            }

            if (int status = nl_connect(mSocket, NETLINK_ROUTE); status < 0) {
                throw std::runtime_error("Failed to connect to the network link socket");
            }

            nl_cache* cache;
            rtnl_link_alloc_cache(mSocket, AF_UNSPEC, &cache);
            if (cache == nullptr) {
                throw std::runtime_error("Failed to allocate the network link cache");
            }

            mLink = rtnl_link_get_by_name(cache, interface.c_str());
            if (mLink == nullptr) {
                throw std::runtime_error(std::format("Failed to retrieve the link {} by name", interface));
            }

            bool is_up = rtnl_link_get_flags(mLink) & IFF_UP;

            if (interface.starts_with("can")) {
                can_bittiming bt{
                        .bitrate = bitrate,
                        .brp = bitrate_prescaler,
                };
                if (int result = rtnl_link_can_set_bittiming(mLink, &bt); result < 0) {
                    throw std::runtime_error("Failed to set CAN bit timing");
                }

                // Trying to send to the socket without this will return error code 22 (invalid argument)
                // By default the MTU is only set for regular CAN frames which are much smaller
                // The effects of these calls will not be realized until "rtnl_link_change"
                rtnl_link_set_mtu(mLink, sizeof(canfd_frame));
                if (is_up) {
                    ROS_WARN("Link is already up");
                } else {
                    rtnl_link_set_flags(mLink, IFF_UP);
                }

                if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                    throw std::runtime_error(std::format("Failed to set CAN link up: {}", result));
                }
            } else if (interface.starts_with("vcan")) {
                if (!is_up) {
                    throw std::runtime_error("Virtual CAN link must be set up manually");
                }
            }

            ROS_INFO_STREAM("Set CAN link up");

        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM("Exception in link setup: " << exception.what());
            ros::shutdown();
        }
    }

    CanNetLink::~CanNetLink() {
        if (!mLink || !mSocket) return;

        rtnl_link_unset_flags(mLink, IFF_UP);
        if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
            std::cerr << std::format("Failed to change link: {}", result) << std::endl;
        }
        mLink = nullptr;

        nl_socket_free(mSocket);
        mSocket = nullptr;
    }

} // namespace mrover
