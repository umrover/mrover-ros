#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

class CanNode {
public:
    CanNode() {
        struct sockaddr_can addr {};
        struct ifreq ifr {};

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Error while opening socket");
        }

        const char* ifname = "can0";
        strcpy(ifr.ifr_name, ifname);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
            perror("Error in socket bind");
        }
    }

    void send_frame() const {
        // write(s, &frame, sizeof(struct can_frame));
        sendto(s, &frame, sizeof(struct can_frame), 0, nullptr, 0);
    }

    void set_bus(uint8_t bus) {
        this->bus = bus;
    }

    void set_frame_id(uint16_t id) {
        uint32_t identifier = static_cast<uint32_t>(id) & 0x000007FF; // can_id[0-10]
        uint32_t errorFrameFlag = 0x20000000;                         // can_id[29]
        uint32_t rtrFlag = 0x40000000;                                // can_id[30]
        uint32_t frameFormatFlag = 0;                                 // can_id[31]

        frame.can_id = identifier | errorFrameFlag | rtrFlag | frameFormatFlag;
    }

    void set_frame_data(size_t len, std::vector<uint8_t> const& data) {
        frame.len = static_cast<uint8_t>(len);
        for (size_t i = 0; i < len; ++i) {
            frame.data[i] = data[i];
        }
    }

private:
    uint8_t bus{};
    struct canfd_frame frame {};
    int s{};

    // Helper function for debug
    void printFrame() {
        std::cout << "BUS: " << unsigned(bus) << "\n";
        std::cout << "CAN_ID: " << frame.can_id << "\n";
        std::cout << "LEN: " << unsigned(frame.len) << "\n";
        std::cout << "DATA:\n";

        for (size_t i = 0; i < static_cast<size_t>(frame.len); ++i) {
            std::cout << "Index = " << i << "\tData = " << unsigned(frame.data[i]) << "\n";
        }
    }
};

// 1 = high = recessive
// 0 = low = dominant

/*
struct canfd_frame {
    uint32_t can_id; // 32 bit CAN_ID + EFF/RTR/ERR flags
                     // [0-28]: CAN identifier (11/29bit)
                     // [29]: Error frame flag (0 = data frame, 1 = error frame)
                     // [30]: Remote transmission request flag (1 = rtr frame)
                     // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)

    uint8_t len;    // frame payload length in byte (0 .. 64)
    uint8_t flags;  // additional flags for CAN FD
    uint8_t __res0; // reserved / padding
    uint8_t __res1; // reserved / padding

    
    // REQUIREMENT: DATA MUST BE 0-8, 12, 16, 20, 24, 32, 36, 48, or 64 BYTES LONG
    uint8_t data[64];
};

CAN Frame Layout:

Start of Frame (SOF): 1 bit

Arbitration Field: 12 bits (for standard ID)

ID: 11 bits
RTR: 1 bit
Control Field: more bits compared to classic CAN due to extended DLC

IDE: 1 bit
r0: 1 bit
Extended Data Length (EDL): 1 bit
Bit Rate Switch (BRS): 1 bit
Error State Indicator (ESI): 1 bit
DLC: 4 bits
Data Field: up to 64 bytes in CAN FD (not limited to 8 bytes as in classic CAN)

CRC Field: variable size depending on the length of the Data Field

CRC Sequence
CRC Delimiter: 1 bit
Acknowledgment Field: 2 bits

ACK Slot: 1 bit
ACK Delimiter: 1 bit
End of Frame (EOF): 7 bits

Intermission: 3 bits

*/