#pragma once

#include "I2C.h"         // for IOFailure
#include <fcntl.h>       // for open and O_RDWR
#include <mutex>         // for std::mutex
#include <ros/console.h> // for ROS_ERROR
#include <stdint.h>      // for uint8_t
#include <string.h>      // for string and memcpy
#include <termios.h>     // for termios
#include <unistd.h>      // for read/write


// Used to abstract UART and hardware related functions
class UART {
private:
    // file stores the UART device file (e.g. /dev/ttyACM0)
    // transactLock ensures that only one uart transaction happens at a time
    inline static int file = -1;
    inline static std::mutex transactLock;

public:
    // REQUIRES: device_file is the UART device file
    // e.g. /dev/ttyACM0, etc.
    // MODIFIES: nothing
    // EFFECTS: Opens the UART bus.
    static void init(std::string& device_file);

    // REQUIRES: addr is the address of the slave,
    // cmd is the register/command,
    // writeNum is number of bytes being written,
    // writeBuf is array of bytes being written,
    // writeBuf must be nullptr if and only if writeNum is 0.
    // 0 <= writeNum <= 31
    // MODIFIES: UART bus. Send the message as:
    // S[ADDR][CMD][WRITE_NUM][WRITE_BUF][E]
    // EFFECTS: Executes a write transaction.
    static void transact(
            const uint8_t addr,
            const uint8_t cmd,
            const uint8_t writeNum,
            uint8_t* writeBuf);
};
