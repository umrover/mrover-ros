#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <mrover/CAN.h>


ros::Publisher CANPublisher;

namespace mrover {
    int main(int argc, char** argv) {
        ros::init(argc, argv, "advertiser_can_node");
        ros::NodeHandle nh;

        CANPublisher = nh.advertise<mrover::CAN>("can_commands", 1);


        int s;
        try {
            if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                throw std::runtime_error("Failed to open socket");
            }

            ifreq ifr{};
            const char* interfaceName = "can0";
            strcpy(ifr.ifr_name, interfaceName);
            ioctl(s, SIOCGIFINDEX, &ifr);

            sockaddr_can addr{
                    .can_family = AF_CAN,
                    .can_ifindex = ifr.ifr_ifindex,
            };


            if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
                throw std::runtime_error("Failed to bind to socket");
            }

            int enable_canfd = 1;
            if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
                throw std::runtime_error("Failed to enable CAN FD");
            }
        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }

        canfd_frame frame{};
        size_t nBytesRead;
        while (true) {
            nBytesRead = read(s, &frame, sizeof(canfd_frame));
            if (nBytesRead == sizeof(canfd_frame)) {
                mrover::CAN msg;
                msg.bus = 0;
                msg.message_id = frame.can_id;
                std::memcpy(msg.data.data(), frame.data, frame.len);

                CANPublisher.publish(msg);
            }
        }

        return EXIT_SUCCESS;
    }
} // namespace mrover