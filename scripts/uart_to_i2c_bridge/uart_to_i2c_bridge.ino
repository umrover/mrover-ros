#define MIN_UART_MESSAGE_LENGTH 5
#define MAX_UART_MESSAGE_LENGTH 36

#define MAX_DATA_LENGTH (MAX_UART_MESSAGE_LENGTH - MIN_UART_MESSAGE_LENGTH)

#define I2C_WRITE_TIMEOUT_MS 500

#include "Wire.h"

void setup() {
  Serial.begin(9600); // Initialize the serial port with baud rate 9600
  while (!Serial) { }
  Serial.setTimeout(10);

  Wire.begin(); // Initialize I2C communication
}


void send_i2c(uint8_t addr, uint8_t write_num, uint8_t* write_buf) {
  // Set the I2C device address based on the UART address
  uint8_t i2c_address = 0;
  if (addr == 1) {
    i2c_address = 0x01;
  } else if (addr == 2) {
    i2c_address = 0x02;
  } else {
    return; // Invalid UART address
  }

  // Send the I2C message to the appropriate device address
  Wire.beginTransmission(i2c_address);
  Wire.write(write_buf, write_num);
  Wire.endTransmission();
}

void read_uart_message() {

  // One byte for command, then data.
  uint8_t[MAX_DATA_LENGTH + 1] data_buf;

  if (Serial.available() >= MIN_UART_MESSAGE_LENGTH) {
    uint8_t first_byte = Serial.read();

    if (first_byte != 'S') {
      // Discard any bytes until we see the start character
      return;
    }

    // Read in address.
    uint8_t addr = Serial.read();

    // Read in command.
    data_buf[0] = Serial.read();

    // Read in the number of data bytes to read.
    uint8_t write_num = Serial.read();

    if (write_num > MAX_DATA_LENGTH) {
      return;
    }
    Serial.readBytes(data_buf + 1, write_num);

    uint8_t last_byte;
    Serial.readBytes(&last_byte, 1);
    if (last_byte != 'E') {
      return;
    }

    // End of message, parse it and reset message_length
    send_i2c(addr, write_num + 1, data_buf)
  }
}

void loop() {
  read_uart_message();
}
