#include "I2C.h"


// REQUIRES: device_file is the i2c device file
// e.g. /dev/i2c-0, /dev/i2c-1, etc.
// MODIFIES: nothing
// RETURNS: nothing
void I2C::init(std::string& device_file) {

    file = open(device_file, O_RDWR);
    if (file == -1) {
        printf("Failed to open I2C bus\n");
        throw IOFailure();
    }
} // I2C::init

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
// RETURNS: nothing
void I2C::transact(
        const uint8_t addr,
        const uint8_t cmd,
        const uint8_t writeNum,
        const uint8_t readNum,
        uint8_t* writeBuf,
        uint8_t* readBuf) {

    // writeBuf must be nullptr if and only if writeNum is 0.
    assert((bool) writeNum == (bool) writeBuf);
    // writeBuf must be nullptr if and only if writeNum is 0.
    assert((bool) readNum == (bool) readNum);
    assert(0 <= writeNum <= 31);
    assert(0 <= readNum <= 32);

            std::unique_lock<std::mutex>
                    lck(transact_m);

    if (file == -1) {
        printf("I2C Port never opened. Make sure to first run I2C::init.");
        throw IOFailure();
    }

    // Copy command and write data into the buffer.
    uint8_t buffer[32];
    buffer[0] = cmd;
    memcpy(buffer + 1, writeBuf, writeNum);

    // Switch to this slave address.
    ioctl(file, I2C_SLAVE, addr);

    // Write bytes and confirm that all bytes were written.
    if (writeNum) {
        if (write(file, buffer, writeNum + 1) != writeNum + 1) {
            fprintf(stderr, "Write error %d\n", errno);
            throw IOFailure();
        }
    }
    // Read bytes and confirm that all bytes were read.
    if (readNum) {
        if (read(file, buffer, readNum) != readNum) {
            fprintf(stderr, "read error %d\n", errno);
            throw IOFailure();
        }
    }

    // Copy data to readBuf.
    memcpy(readBuf, buffer, readNum);
} // I2C::transact