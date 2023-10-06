#include "CanNode.hpp"

CanNode::CanNode() {
    struct sockaddr_can addr{};
    struct ifreq ifr{};

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
    }

    const char *ifname = "can0";
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
    }
}

void CanNode::sendFrame(uint8_t bus, uint16_t id, size_t dataLength, std::vector<uint8_t>& data) {
    this->bus = bus;

    uint32_t identifier = ((uint32_t) id) & 0x000007FF;
    uint32_t errorFrameFlag = 0x20000000;
    uint32_t rtrFlag = 0x40000000;
    uint32_t frameFormatFlag = 0;

    frame.can_id = identifier | errorFrameFlag | rtrFlag | frameFormatFlag;
    frame.len = (uint8_t) dataLength;
    for (size_t i = 0; i < dataLength; ++i) {
        frame.data[i] = data[i];
    }

    // write(s, &frame, sizeof(struct can_frame));
    sendto(s, &frame, sizeof(struct can_frame), 0, nullptr, 0);
}

void CanNode::printFrame() {
    std::cout << "BUS: " << unsigned(bus) << "\n";
    std::cout << "CAN_ID: " << frame.can_id << "\n";
    std::cout << "LEN: " << unsigned(frame.len) << "\n";
    std::cout << "DATA:\n";

    for (size_t i = 0; i < static_cast<size_t>(frame.len); ++i) {
        std::cout << "Index = " << i << "\tData = " << unsigned(frame.data[i]) << "\n";
    }
}