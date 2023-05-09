#include "UART.h"

// REQUIRES: device_file is the UART device file
// e.g. /dev/ttyACM0, etc.
// MODIFIES: nothing
// EFFECTS: Opens the UART bus.
void UART::init(std::string& device_file) {
    file = open(device_file.c_str(), O_RDWR);
    if (file == -1) {
        ROS_ERROR("Failed to open UART bus\n");
        throw IOFailure();
    }

    // termios struct is used to configure the UART port
    struct termios tty{};
    tcgetattr(file, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tcsetattr(file, TCSANOW, &tty);
}

// REQUIRES: addr is the address of the slave,
// cmd is the register/command,
// writeNum is number of bytes being written,
// writeBuf is array of bytes being written,
// writeBuf must be nullptr if and only if writeNum is 0.
// 0 <= writeNum <= 31
// MODIFIES: UART bus. Send the message as:
// S[ADDR][CMD][WRITE_NUM][WRITE_BUF][E]
// EFFECTS: Executes a write transaction.
void UART::transact(
        const uint8_t addr,
        const uint8_t cmd,
        const uint8_t writeNum,
        uint8_t* writeBuf) {

    // writeBuf must be nullptr if and only if writeNum is 0.
    assert((bool) writeNum == (bool) writeBuf);
    assert(writeNum <= 31);

    std::unique_lock<std::mutex>
            lck(transactLock);

    if (file == -1) {
        ROS_ERROR("UART Port never opened. Make sure to first run UART::init.");
        throw IOFailure();
    }

    // Copy command and write data into the buffer.
    uint8_t buffer[36];

    // "S[ADDR][CMD][WRITE_NUM][WRITE_BUF]E" is 5 + WRITE_NUM
    // Everything except the write buffer is 1 byte.
    int uart_data_bytes_sending = 5 + writeNum;

    buffer[0] = 'S';
    buffer[1] = addr;
    buffer[2] = cmd;
    buffer[3] = writeNum;
    memcpy(buffer + 4, writeBuf, writeNum);
    buffer[4 + writeNum] = 'E';

    int bitsWritten = (int) write(file, buffer, sizeof(uart_data_bytes_sending));

    if (bitsWritten != uart_data_bytes_sending) {
        ROS_ERROR("Write error %d, wrote %i bits", errno, bitsWritten);
        throw IOFailure();
    }

}
