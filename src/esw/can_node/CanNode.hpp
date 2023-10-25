#pragma once

#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <vector>
#include <iostream>
#include <cstring>

const size_t FRAME_SIZE = 64;

// true = high = recessive
// false = low = dominant

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
*/

class CanNode {
public:
    CanNode();
    void sendFrame(uint8_t bus, uint16_t id, size_t dataLength, std::vector<uint8_t>& data);


private:
    uint8_t bus{};
    struct canfd_frame frame{};
    int s{};

    // Helper function for debug
    void printFrame();
};

/* CAN Frame Layout

|SOF|---Identifier Bits---|-- ... --|--------   Data Frame (hex) -----------|
| 0 |x|x|x|x|x|x|x| | | | |   ...   | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
|-------------------------|-- ... --| 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    | 00 | 00 | 00 | 00 | 00 | 00 | 00 | 00 |
                                    |---------------------------------------|

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