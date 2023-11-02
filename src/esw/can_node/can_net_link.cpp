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
                throw std::runtime_error("Failed to allocate netlink socket");
            }

            if (int status = nl_connect(mSocket, NETLINK_ROUTE); status < 0) {
                throw std::runtime_error("Failed to connect to netlink socket");
            }

            nl_cache* cache;
            rtnl_link_alloc_cache(mSocket, AF_UNSPEC, &cache);
            if (cache == nullptr) {
                throw std::runtime_error("Failed to allocate rtnl_link cache");
            }

            mLink = rtnl_link_get_by_name(cache, interface.c_str());
            if (mLink == nullptr) {
                throw std::runtime_error("Failed to get rtnl_link");
            }

            can_bittiming bt{
                    .bitrate = bitrate,
                    .brp = bitrate_prescaler,
            };
            if (int result = rtnl_link_can_set_bittiming(mLink, &bt); result < 0) {
                throw std::runtime_error("Failed to set bittiming");
            }

            rtnl_link_set_flags(mLink, IFF_UP);
            if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                throw std::runtime_error(std::format("Failed to change rtnl_link: {}", result));
            }

            ROS_INFO_STREAM("Set CAN socket up");

        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM("Exception in link setup: " << exception.what());
            ros::shutdown();
        }
    }

    CanNetLink::~CanNetLink() {
        try {
            rtnl_link_unset_flags(mLink, IFF_UP);
            if (int result = rtnl_link_change(mSocket, mLink, mLink, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                throw std::runtime_error(std::format("Failed to change rtnl_link: {}", result));
            }

            nl_socket_free(mSocket);

        } catch (std::exception const& exception) {
            std::cerr << std::format(
                    "Exception in link cleanup:  {}\n", exception.what());
        }
    }

}; // namespace mrover