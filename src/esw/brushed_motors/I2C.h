#pragma once

#include <assert.h>        // for assert
#include <exception>       // for std::exception
#include <fcntl.h>         // for open and O_RDWR
#include <linux/i2c-dev.h> // for I2C_SLAVE
#include <mutex>           // for std::mutex
#include <ros/console.h>   // for ROS_INFO
#include <stdint.h>        // for uint
#include <string.h>        // for string and memcpy
#include <sys/ioctl.h>     // for ioctl
#include <unistd.h>        // for read/write


// An exception through when a read or write transaction fails.
struct IOFailure : public std::exception {};


// Used to abstract I2C and hardware related functions
class I2C {
private:
    // file stores the i2c device file (e.g. /dev/i2c-0, /dev/i2c-1)
    // transact_m ensures that only one i2c transaction happens at a time
    inline static int file = -1;
    inline static std::mutex transactLock;

public:
    // REQUIRES: device_file is the i2c device file
    // e.g. /dev/i2c-0, /dev/i2c-1, etc.
    // MODIFIES: nothing
    // EFFECTS: Opens the I2C bus.
    static void init(std::string& device_file);

    // REQUIRES: addr is the address of the slave,
    // cmd is the register/command,
    // writeNum is number of bytes being written,
    // readNum is number of bytes being read,
    // writeBuf is array of bytes being written,
    // and readBuf is array of bytes that will store read values.
    // writeBuf must be nullptr if and only if writeNum is 0.
    // readBuf must be nullptr if and only if readNum is 0.
    // 0 <= writeNum <= 31 and 0 <= readNum <= 32.
    // MODIFIES: readBuf
    // EFFECTS: Executes a read and/or write transaction.
    static void transact(
            const uint8_t addr,
            const uint8_t cmd,
            const uint8_t writeNum,
            const uint8_t readNum,
            uint8_t* writeBuf,
            uint8_t* readBuf);
};
