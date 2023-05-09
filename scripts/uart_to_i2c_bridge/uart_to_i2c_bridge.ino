#define MAX_UART_MESSAGE_LENGTH 36
#define I2C_WRITE_TIMEOUT_MS 500

void setup() {
  Serial.begin(9600); // Initialize the serial port with baud rate 9600
  Wire.begin(); // Initialize I2C communication
}


void convert_uart_to_i2c(uint8_t addr, uint8_t cmd, uint8_t write_num, uint8_t* write_buf) {
  // Set the I2C device address based on the UART address
  uint8_t i2c_address = 0;
  if (addr == 1) {
    i2c_address = 0x01;
  } else if (addr == 2) {
    i2c_address = 0x02;
  } else {
    return; // Invalid UART address
  }

  // Create an I2C message buffer with the appropriate data
  uint8_t i2c_message_buffer[write_num + 1];
  i2c_message_buffer[0] = cmd;
  memcpy(&i2c_message_buffer[1], write_buf, write_num);

  // Send the I2C message to the appropriate device address
  Wire.beginTransmission(i2c_address);
  Wire.write(i2c_message_buffer, write_num + 1);
  Wire.endTransmission();
}

void parse_uart_message(uint8_t* uart_message, uint8_t message_length) {
  // Check that the message has the correct length
  if (message_length < 5) {
    return; // Invalid message length
  }

  // Check the start and end characters of the message
  if (uart_message[0] != 'S' || uart_message[message_length - 1] != 'E') {
    return; // Invalid start or end character
  }

  // Parse the message fields
  uint8_t addr = uart_message[1];
  uint8_t cmd = uart_message[2];
  uint8_t write_num = uart_message[3];
  uint8_t* write_buf = &uart_message[4];

  // Check that the write buffer has the correct length
  if (message_length != 5 + write_num) {
    return; // Invalid write buffer length
  }

  // Convert the UART message to an I2C message and send it
  convert_uart_to_i2c(addr, cmd, write_num, write_buf);
}

void read_uart_message() {
  static uint8_t uart_message[MAX_UART_MESSAGE_LENGTH];
  static uint8_t message_length = 0;

  while (Serial.available() > 0) {
    uint8_t byte_read = Serial.read();

    if (message_length == 0 && byte_read != 'S') {
      // Discard any bytes until we see the start character
      continue;
    }

    if (message_length >= MAX_UART_MESSAGE_LENGTH) {
      // Discard any bytes if the message is too long
      message_length = 0;
      continue;
    }

    uart_message[message_length] = byte_read;
    message_length++;

    if (byte_read == 'E') {
      // End of message, parse it and reset message_length
      parse_uart_message(uart_message, message_length);
      message_length = 0;
    }
  }
}

void loop() {
  read_uart_message();
}

