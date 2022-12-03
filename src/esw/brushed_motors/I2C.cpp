#include "I2C.h"

// REQUIRES: device_file is the i2c device file
// e.g. /dev/i2c-0, /dev/i2c-1, etc.
// MODIFIES: nothing
// EFFECTS: Opens the I2C bus.
void I2C::init(std::string& device_file) {
    file = open(device_file.c_str(), O_RDWR);
    if (file == -1) {
        ROS_ERROR("Failed to open I2C bus\n");
        throw IOFailure();
    }
}

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
    assert(0 <= writeNum);
    assert(writeNum <= 31);
    assert(0 <= readNum);
    assert(readNum <= 32);

    std::unique_lock<std::mutex>
            lck(transactLock);

    if (file == -1) {
        ROS_ERROR("I2C Port never opened. Make sure to first run I2C::init.");
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
        int bitsWritten = (int) write(file, buffer, writeNum + 1);
        if (bitsWritten != writeNum + 1) {
            ROS_ERROR("Write error %d, wrote %i bits", errno, bitsWritten);
            throw IOFailure();
        }
    }
    // Read bytes and confirm that all bytes were read.
    if (readNum) {
        int bitsRead = (int) read(file, buffer, readNum);
        if (bitsRead != readNum) {
            ROS_ERROR("read error %d, read %i bits", errno, bitsRead);
            throw IOFailure();
        }
    }

    // Copy data to readBuf.
    memcpy(readBuf, buffer, readNum);
}