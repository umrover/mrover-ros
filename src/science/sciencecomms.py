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

# Mapping of onboard devices to mosfet devices
mosfet_dev_map = {
    "arm_laser": 1,
    "heater_0": 7,
    "heater_1": 8,
    "heater_2": 9,
    "uv_led_carousel": 4,
    "uv_led_end_effector": 1,
    "white_led": 5
}

# Mapping of device color to number
led_map = {
    "Red": 0,
    "Blue": 1,
    "Green": 2,
    "Off": 3
}


def add_padding(tx_msg: str) -> str:
    """Used to add padding since UART messages must be of certain length"""
    while len(tx_msg) < UART_TRANSMIT_MSG_LEN:
        tx_msg += ","
    return tx_msg


def format_mosfet_msg(device: int, enable: bool) -> str:
    """Formats a mosfet message"""
    tx_msg = f"$MOSFET,{device},{enable}"
    return tx_msg


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


def send_mosfet_msg(device_name: str, enable: bool) -> None:
    """Transmits a mosfet device state command message"""
    translated_device = mosfet_dev_map[device_name]
    tx_msg = format_mosfet_msg(translated_device, int(enable))
    send_msg(tx_msg)


def send_msg(tx_msg: str) -> None:
    """Transmits a string over UART of proper length"""
    try:
        tx_msg = add_padding(tx_msg)
        if len(tx_msg) > UART_TRANSMIT_MSG_LEN:
            tx_msg = tx_msg[:UART_TRANSMIT_MSG_LEN]
        print(tx_msg)
        ser.open()
        ser.write(bytes(tx_msg, encoding='utf-8'))
        ser.close()
    except serial.SerialException as exc:
        print("send_msg exception:", exc)
