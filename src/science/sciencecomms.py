import serial

# import Adafruit_BBIO.UART as UART  # for beaglebone use only
# UART.setup("UART4")  # Specific to beaglebone

UART_TRANSMIT_MSG_LEN = 30
ser = serial.Serial(
        port='/dev/ttyTHS0',
        baudrate=38400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0
    )
ser.close()


def add_padding(uart_msg: str) -> str:
    while(len(uart_msg) < UART_TRANSMIT_MSG_LEN):
        uart_msg += ","
    return uart_msg


def uart_send(uart_msg: str) -> None:
    uart_msg = add_padding(uart_msg)
    if len(uart_msg) > UART_TRANSMIT_MSG_LEN:
        uart_msg = uart_msg[:UART_TRANSMIT_MSG_LEN]
    print(uart_msg)
    ser.open()
    ser.write(bytes(uart_msg, encoding='utf-8'))
    ser.close()

def read_msg() -> str:
    ser.open()
    tx = ser.readline()
    ser.close()
    return str(tx)

