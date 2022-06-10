#ifndef I2C_H
#define I2C_H

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <exception>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <mutex>


struct IOFailure : public std::exception {};

class I2C
{
private:
    inline static int file = -1;
    inline static std::mutex transact_m;

public:
    //Abstraction for I2C/Hardware related functions
    static void init();

    //Performs an i2c transaction
    static void transact(uint8_t addr, uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf);
};

#endif
