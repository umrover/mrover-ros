'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
# import Adafruit_BBIO.UART as UART  # for beaglebone use only
import numpy as np
import rospy
import serial
from mrover import AutonLED, Enable, Heater, Servo, Spectral, Thermistor


class ScienceBridge():

    def __init__(self):

        # UART.setup("UART4")  # Specific to beaglebone

        self.NMEA_HANDLE_MAPPER = {
            "AUTOSHUTOFF": self.heater_auto_shut_off_handler,
            "HEATER": self.heater_state_handler,
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler
        }
        self.NMEA_PUBLISHER_MAPPER = {
            "AUTOSHUTOFF": rospy.Publisher('heater_auto_shut_off_data', Enable, queue_size=1),
            "HEATER": rospy.Publisher('heater_state_data', Heater, queue_size=1),
            "SPECTRAL": rospy.Publisher('spectral_data', Spectral, queue_size=1),
            "THERMISTOR": rospy.Publisher('thermistor_data', Thermistor, queue_size=1),
            "TRIAD": rospy.Publisher('spectral_triad_data', Spectral, queue_size=1)
        }
        self.NMEA_MESSAGE_MAPPER = {
            "AUTOSHUTOFF": Enable(),
            "HEATER": Heater(),
            "SPECTRAL": Spectral(),
            "THERMISTOR": Thermistor(),
            "TRIAD": Spectral()
        }
        self.max_error_count = 20
        self.sleep = .01
        self.led_map = {
            "Red": 0,
            "Blue": 1,
            "Green": 2,
            "Off": 3
        }
        # Mapping of onboard devices to mosfet devices
        self.mosfet_dev_map = {
            "arm_laser": 1,
            "raman_laser": 10,
            "uv_bulb": 1,
            "uv_led": 4,
            "white_led": 5
        }
        self.heater_map = [7, 8, 9]
        self.UART_TRANSMIT_MSG_LEN = 30

    def __enter__(self) -> None:
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            port='/dev/ttyTHS0',
            baudrate=38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()

    def add_padding(self, uart_msg: str) -> str:
        while(len(uart_msg) < self.UART_TRANSMIT_MSG_LEN):
            uart_msg += ","
        return uart_msg

    def auton_led_transmit(self, ros_msg: AutonLED) -> None:
        try:
            requested_state = self.led_map[ros_msg.color]
            print("Received new auton led request: Turning " + str(ros_msg.color))
        except KeyError:
            requested_state = self.led_map["Off"]
            print("Received invalid/off auton led request: Turning off all colors")

        uart_msg = "$LED,{led_color}".format(led_color=requested_state.value)
        self.uart_send(uart_msg)

    def format_mosfet_msg(self, device: int, enable: bool) -> str:
        uart_msg = "$MOSFET,{dev},{en},"
        return uart_msg.format(dev=device, en=enable)

    def heater_auto_shut_off_transmit(self, ros_msg: Enable) -> None:
        uart_msg = "$AUTOSHUTOFF,{enable}"
        uart_msg = uart_msg.format(enable=int(ros_msg.auto_shut_off_enabled))
        self.uart_send(uart_msg)

    def heater_auto_shut_off_handler(self, uart_msg: str, ros_msg: Enable) -> None:
        # uart_msg format: <"$AUTOSHUTOFF,device,enabled">
        try:
            arr = uart_msg.split(",")
            enabled = bool(int(arr[1]))
            ros_msg.enable = enabled
        except Exception as e:
            print(e)

    def heater_state_handler(self, uart_msg: str, ros_msg: Heater) -> None:
        # uart_msg format: <"$HEATER,device,enabled">
        try:
            arr = uart_msg.split(",")
            ros_msg.device = int(arr[1])
            ros_msg.enable = bool(int(arr[2]))
        except Exception as e:
            print(e)

    def heater_transmit(self, ros_msg: Heater) -> None:
        translated_device = self.heater_map[ros_msg.device]
        uart_msg = self.format_mosfet_msg(translated_device, int(ros_msg.enable))
        self.uart_send(uart_msg)

    def mosfet_transmit(self, ros_msg: Enable, device_name: str) -> None:
        translated_device = self.mosfet_dev_map[device_name]
        uart_msg = self.format_mosfet_msg(translated_device, int(ros_msg.enable))
        self.uart_send(uart_msg)

    def servo_transmit(self, ros_msg: Servo) -> None:
        uart_msg = "$SERVO,{angle_0},{angle_1},{angle_2}"
        uart_msg = uart_msg.format(angle_0=ros_msg.angle_0, angle_1=ros_msg.angle_1, angle_2=ros_msg.angle_2)
        self.uart_send(uart_msg)

    def spectral_handler(self, m: str, ros_msg: Spectral) -> None:
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                 "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                 "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]
            # There are 3 spectral sensors, each having 6 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in ros_msg_variables:
                if (not (count >= len(arr))):
                    setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count]) << 8) | (np.uint8(arr[count + 1]))))
                else:
                    setattr(ros_msg, var, 0)
                count += 2
        except Exception as e:
            print("spectral exception:", e)
            pass

    def thermistor_handler(self, uart_msg: str, ros_msg: Thermistor) -> None:
        try:
            arr = uart_msg.split(",")
            ros_msg.temp_0 = float(arr[1])
            ros_msg.temp_1 = float(arr[2])
            ros_msg.temp_2 = float(arr[3])
        except Exception as e:
            print("thermistor exception:", e)

    def triad_handler(self, m: str, ros_msg: Spectral) -> None:
        try:

            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            ros_msg_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                 "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                 "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

            # There are 18 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in ros_msg_variables:
                if (not (count >= len(arr))):
                    pass
                    setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count + 1]) << 8) | (np.uint8(arr[count]))))
                else:
                    setattr(ros_msg, var, 0)
                count += 2
        except Exception as e:
            print("triad exception:", e)
            pass

    def uart_send(self, uart_msg: str) -> None:
        uart_msg = self.add_padding(uart_msg)
        print(uart_msg)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(uart_msg, encoding='utf-8'))

    def receive(self) -> None:
        while True:
            try:
                error_counter = 0
                tx = self.ser.readline()
                uart_msg = str(tx)
            except Exception as e:
                print("Errored")
                if error_counter < self.max_error_count:
                    error_counter += 1
                    print(e)
                    rospy.sleep(self.sleep)
                    continue
                else:
                    raise e
            match_found = False
            for tag, handler_func in self.NMEA_HANDLE_MAPPER.items():
                if tag in uart_msg:  # TODO - why do we have tag in uart_msg, func is not even used
                    print(uart_msg)
                    match_found = True
                    try:
                        handler_func(uart_msg, self.NMEA_MESSAGE_MAPPER[tag])
                        self.NMEA_PUBLISHER_MAPPER[tag].publish(self.NMEA_MESSAGE_MAPPER[tag])
                    except Exception as e:
                        print(e)
                    break
            if not match_found:
                if not uart_msg:
                    print('Error decoding message stream: {}'.format(uart_msg))
            rospy.sleep(self.sleep)


def main():

    rospy.init_node("science")

    with ScienceBridge() as bridge:
        rospy.Subscriber("auton_led", AutonLED, bridge.auton_led_transmit)
        rospy.Subscriber("arm_laser_cmd", Enable, bridge.mosfet_transmit, "arm_laser")
        rospy.Subscriber("heater_cmd", Heater, bridge.heater_transmit)
        rospy.Subscriber("heater_auto_shut_off_cmd", Enable, bridge.heater_auto_shut_off_transmit)
        rospy.Subscriber("raman_laser_cmd", Enable, bridge.mosfet_transmit, "raman_laser")
        rospy.Subscriber("servo_cmd", Servo, bridge.servo_transmit)
        rospy.Subscriber("uv_bulb_cmd", Enable, bridge.mosfet_transmit, "uv_bulb")
        rospy.Subscriber("uv_led_cmd", Enable, bridge.mosfet_transmit, "uv_led")
        rospy.Subscriber("white_led_cmd", Enable, bridge.mosfet_transmit, "white_led")

        while not rospy.is_shutdown():
            bridge.receive()


if __name__ == "__main__":
    main()
