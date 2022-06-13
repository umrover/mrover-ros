"""Import serial since UART is used"""
import serial

# import Adafruit_BBIO.UART as UART  # for beaglebone use only
# UART.setup("UART4")  # Specific to beaglebone

UART_TRANSMIT_MSG_LEN = 30
MAX_ERROR_COUNT = 20
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
    """Used to add padding since UART messages must be of certain length"""
    while len(uart_msg) < UART_TRANSMIT_MSG_LEN:
        uart_msg += ","
    return uart_msg


def send_msg(uart_msg: str) -> None:
    """Transmits a string over UART of proper length"""
    try:
        uart_msg = add_padding(uart_msg)
        if len(uart_msg) > UART_TRANSMIT_MSG_LEN:
            uart_msg = uart_msg[:UART_TRANSMIT_MSG_LEN]
        print(uart_msg)
        ser.open()
        ser.write(bytes(uart_msg, encoding='utf-8'))
        ser.close()
    except serial.SerialException as exc:
        print("send_msg exception:", exc)


def read_msg() -> str:
    """Used to read a message on the UART receive line"""
    error_counter = 0
    try:
        ser.open()
        msg = ser.readline()
        ser.close()
        return str(msg)
    except serial.SerialException as exc:
        print("Errored")
        if error_counter < MAX_ERROR_COUNT:
            error_counter += 1
            print(exc)
        else:
            raise exc
