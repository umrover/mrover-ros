#include "can_net_link.hpp"

#include <format>
#include <stdexcept>

#include <net/if.h>

#include <ros/init.h>

#include <netlink/route/link.h>

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
                throw std::runtime_error(std::format("Failed to retrieve the network link {} by name", interface));
            }

            if (interface.starts_with("can")) {
                can_bittiming bt{
                        .bitrate = bitrate,
                        .brp = bitrate_prescaler,
                };
                if (int result = rtnl_link_can_set_bittiming(mLink, &bt); result < 0) {
                    throw std::runtime_error("Failed to set bit timing");
                }
            }

            uint flags = rtnl_link_get_flags(mLink);
            if (flags & IFF_UP) {
                ROS_WARN("Network link is already up");
            } else {
                rtnl_link_set_flags(mLink, IFF_UP);
                if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                    throw std::runtime_error(std::format("Failed to set network link up: {}", result));
                }
            }

            ROS_INFO_STREAM("Set CAN socket up");

        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM("Exception in link setup: " << exception.what());
            ros::shutdown();
        }
    }

    CanNetLink::~CanNetLink() {
        if (!mLink || !mSocket) return;

        try {
            rtnl_link_unset_flags(mLink, IFF_UP);
            if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                throw std::runtime_error(std::format("Failed to set network link down: {}", result));
            }
            mLink = nullptr;

            nl_socket_free(mSocket);
            mSocket = nullptr;

        } catch (std::exception const& exception) {
            // Can not use ROS logging here because ROS is already shutdown
            std::cerr << std::format("Exception in network link cleanup: {}", exception.what()) << std::endl;
        }
    }

} // namespace mrover
