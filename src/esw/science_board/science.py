'''
Writes, reads and parses NMEA like messages from the onboard
science Nucleo to operate the science boxes and get relevant data
'''
import threading

import numpy as np
import rospy
import serial
from mrover.msg import Enable, Heater, Spectral, Thermistor
from mrover.srv import (ChangeAutonLEDState, ChangeAutonLEDStateRequest,
                        ChangeAutonLEDStateResponse, ChangeDeviceState,
                        ChangeDeviceStateRequest, ChangeDeviceStateResponse,
                        ChangeHeaterState, ChangeHeaterStateRequest,
                        ChangeHeaterStateResponse, ChangeServoAngles,
                        ChangeServoAnglesRequest, ChangeServoAnglesResponse)


class ScienceBridge():
    """ScienceBridge class"""
    def __init__(self):
        self.nmea_handle_mapper = {
            "AUTOSHUTOFF": self.heater_auto_shut_off_handler,
            "HEATER": self.heater_state_handler,
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler
        }
        self.nmea_publisher_mapper = {
            "AUTOSHUTOFF": rospy.Publisher(
                'heater_auto_shut_off_data', Enable, queue_size=1),
            "HEATER": rospy.Publisher(
                'heater_state_data', Heater, queue_size=1),
            "SPECTRAL": rospy.Publisher(
                'spectral_data', Spectral, queue_size=1),
            "THERMISTOR": rospy.Publisher(
                'thermistor_data', Thermistor, queue_size=1),
            "TRIAD": rospy.Publisher(
                'spectral_triad_data', Spectral, queue_size=1)
        }
        self.nmea_message_mapper = {
            "AUTOSHUTOFF": Enable(),
            "HEATER": Heater(),
            "SPECTRAL": Spectral(),
            "THERMISTOR": Thermistor(),
            "TRIAD": Spectral()
        }
        self.UART_TRANSMIT_MSG_LEN = 30
        self.MAX_ERROR_COUNT = 20
        # Mapping of onboard devices to mosfet devices
        self.mosfet_dev_map = {
            "arm_laser": 1,
            "heater_0": 7,
            "heater_1": 8,
            "heater_2": 9,
            "uv_led_carousel": 4,
            "uv_led_end_effector": 1,
            "white_led": 5
        }
        # Mapping of device color to number
        self.led_map = {
            "Red": 0,
            "Blue": 1,
            "Green": 2,
            "Off": 3
        }
        # Mapping of heater number to name
        self.heater_map = {
            0: "heater_0",
            1: "heater_1",
            2: "heater_2"
        }
        self.sleep = .01
        self.UART_LOCK = threading.Lock()

    def __enter__(self) -> None:
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            # port='/dev/ttyS4',
            # port='/dev/ttyTHS1',  # used on science nano
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

    def add_padding(self, tx_msg: str) -> str:
        """Used to add padding since UART messages must be of certain length"""
        while len(tx_msg) < self.UART_TRANSMIT_MSG_LEN:
            tx_msg += ","
        return tx_msg

    def auton_led_transmit(self, color: str) -> bool:
        """Sends an auton LED command message via UART"""
        try:
            requested_state = self.led_map[color]
        except KeyError:
            # Done if an invalid color is sent
            return False

        msg = f"$LED,{requested_state.value}"
        success = self.send_msg(msg)
        return success

    def format_mosfet_msg(device: int, enable: bool) -> str:
        """Formats a mosfet message"""
        tx_msg = f"$MOSFET,{device},{enable}"
        return tx_msg

    def handle_change_arm_laser_state(
            self, req: ChangeDeviceStateRequest) -> \
            ChangeDeviceStateResponse:
        """Handle/callback for changing arm laser state service"""
        success = self.send_mosfet_msg("arm_laser", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_auton_led_state(
            self, req: ChangeAutonLEDStateRequest) -> \
            ChangeAutonLEDStateResponse:
        """Handle/callback for changing auton LED state service"""
        success = self.auton_led_transmit(req.color)
        return ChangeAutonLEDStateResponse(success)

    def handle_change_heater_auto_shut_off_state(
            self, req: ChangeDeviceStateRequest) -> \
            ChangeDeviceStateResponse:
        """Handle/callback for changing heater auto shut off state service"""
        success = self.heater_auto_shut_off_transmit(req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_heater_state(
            self, req: ChangeHeaterStateRequest) -> \
            ChangeHeaterStateResponse:
        """Handle/callback for changing heater state service"""
        success = self.heater_transmit(req.device, req.color)
        return ChangeHeaterStateResponse(success)

    def handle_change_servo_angles(
            self, req: ChangeServoAnglesRequest) -> \
            ChangeServoAnglesResponse:
        """Handle/callback for changing servo angles service"""
        success = self.servo_transmit(req.angle_0, req.angle_1, req.angle_2)
        return ChangeServoAnglesResponse(success)

    def handle_change_uv_led_carousel_state(
            self, req: ChangeDeviceStateRequest) -> \
            ChangeDeviceStateResponse:
        """Handle/callback for changing UV LED carousel state service"""
        success = self.send_mosfet_msg("uv_led_carousel", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_uv_led_end_effector_state(
            self, req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
        """Handle/callback for changing UV LED end effector state service"""
        success = self.send_mosfet_msg("uv_led_end_effector", req.enable)
        return ChangeDeviceStateResponse(success)

    def handle_change_white_led_state(
            self, req: ChangeDeviceStateRequest) -> \
            ChangeDeviceStateResponse:
        """Handle/callback for changing white LED state service"""
        success = self.send_mosfet_msg("white_led", req.enable)
        return ChangeDeviceStateResponse(success)

    def heater_auto_shut_off_handler(
            self, tx_msg: str, ros_msg: Enable) -> None:
        """Handles a received heater auto shut off message"""
        # tx_msg format: <"$AUTOSHUTOFF,device,enabled">
        arr = tx_msg.split(",")
        enabled = bool(int(arr[1]))
        ros_msg.enable = enabled

    def heater_auto_shut_off_transmit(self, enable: bool) -> bool:
        """Transmits a heater auto shut off command over uart"""
        tx_msg = f"$AUTOSHUTOFF,{enable}"
        success = self.send_msg(tx_msg)
        return success

    def heater_state_handler(
            self, tx_msg: str, ros_msg: Heater) -> None:
        """Handles a received heater state message"""
        # tx_msg format: <"$HEATER,device,enabled">
        arr = tx_msg.split(",")
        ros_msg.device = int(arr[1])
        ros_msg.enable = bool(int(arr[2]))

    def heater_transmit(self, device: int, enable: bool) -> bool:
        """Sends a heater state command message via UART"""
        heater_device_string = self.heater_map[device]
        translated_device = self.mosfet_dev_map[heater_device_string]
        tx_msg = self.format_mosfet_msg(translated_device, int(enable))
        success = self.send_msg(tx_msg)
        return success

    def read_msg(self) -> str:
        """Used to read a message on the UART receive line"""
        error_counter = 0
        try:
            self.UART_LOCK.acquire()
            msg = self.ser.readline()
            self.UART_LOCK.release()
            return str(msg)
        except serial.SerialException as exc:
            if self.UART_LOCK.locked():
                self.UART_LOCK.release()
            print("Errored")
            if error_counter < self.MAX_ERROR_COUNT:
                error_counter += 1
                print(exc)
            else:
                raise exc

    def send_mosfet_msg(self, device_name: str, enable: bool) -> bool:
        """Transmits a mosfet device state command message"""
        translated_device = self.mosfet_dev_map[device_name]
        tx_msg = self.format_mosfet_msg(translated_device, int(enable))
        success = self.send_msg(tx_msg)
        return success

    def send_msg(self, tx_msg: str) -> bool:
        """Transmits a string over UART of proper length"""
        try:
            tx_msg = self.add_padding(tx_msg)
            if len(tx_msg) > self.UART_TRANSMIT_MSG_LEN:
                tx_msg = tx_msg[:self.UART_TRANSMIT_MSG_LEN]
            print(tx_msg)
            self.UART_LOCK.acquire()
            self.ser.close()
            self.ser.open()
            self.ser.write(bytes(tx_msg, encoding='utf-8'))
            self.UART_LOCK.release()
            return True
        except serial.SerialException as exc:
            if self.UART_LOCK.locked():
                self.UART_LOCK.release()
            print("send_msg exception:", exc)
        return False

    def servo_transmit(
            self, angle_0: float, angle_1: float, angle_2: float) -> bool:
        """Transmits a servo angle command message"""
        tx_msg = f"$SERVO,{angle_0},{angle_1},{angle_2}"
        success = self.send_msg(tx_msg)
        return success

    def spectral_handler(
            self, msg: str, ros_msg: Spectral) -> None:
        """Handles a received spectral data message"""
        msg.split(',')
        arr = [s.strip().strip('\x00') for s in msg.split(',')]
        ros_msg_variables = ["d0_0", "d0_1", "d0_2", "d0_3", "d0_4", "d0_5",
                             "d1_0", "d1_1", "d1_2", "d1_3", "d1_4", "d1_5",
                             "d2_0", "d2_1", "d2_2", "d2_3", "d2_4", "d2_5"]
        # There are 3 spectral sensors, each having 6 channels.
        # We read a uint16_t from each channel.
        # The jetson reads byte by byte, so the program
        # combines every two bytes of information
        # into a uint16_t.
        count = 1
        for var in ros_msg_variables:
            if not count >= len(arr):
                setattr(ros_msg, var, 0xFFFF & ((np.uint8(arr[count]) << 8) |
                        (np.uint8(arr[count + 1]))))
            else:
                setattr(ros_msg, var, 0)
            count += 2

    def thermistor_handler(
            self, tx_msg: str, ros_msg: Thermistor) -> None:
        """Handles a receieved thermistor data message"""
        arr = tx_msg.split(",")
        ros_msg.temp_0 = float(arr[1])
        ros_msg.temp_1 = float(arr[2])
        ros_msg.temp_2 = float(arr[3])

    def triad_handler(
            self, msg: str, ros_msg: Spectral) -> None:
        """Handles a received triad data message"""
        msg.split(',')
        arr = [s.strip().strip('\x00') for s in msg.split(',')]
        ros_msg_variables = ["d0_0", "d0_1", "d0_2", "d0_3", "d0_4", "d0_5",
                             "d1_0", "d1_1", "d1_2", "d1_3", "d1_4", "d1_5",
                             "d2_0", "d2_1", "d2_2", "d2_3", "d2_4", "d2_5"]

        # There are 18 channels.
        # We read a uint16_t from each channel.
        # The jetson reads byte by byte, so the program
        # combines every two byte of information
        # into a uint16_t.
        count = 1
        for var in ros_msg_variables:
            if not count >= len(arr):
                setattr(ros_msg, var, 0xFFFF & (
                    (np.uint8(arr[count + 1]) << 8) |
                    (np.uint8(arr[count]))))
            else:
                setattr(ros_msg, var, 0)
            count += 2

    def receive(self) -> None:
        """Reads in a message from the UART RX line and processes it"""
        tx_msg = self.read_msg()
        match_found = False
        for tag, handler_func in self.nmea_handle_mapper.items():
            if tag in tx_msg:
                print(tx_msg)
                match_found = True
                handler_func(tx_msg, self.nmea_message_mapper[tag])
                self.nmea_publisher_mapper[tag].publish(
                    self.nmea_message_mapper[tag])
                break
        if (not match_found) and (not tx_msg):
            print(f'Error decoding message stream: {tx_msg}')
        rospy.sleep(self.sleep)


def main():
    """Main function"""
    rospy.init_node("science")

    with ScienceBridge() as bridge:
        rospy.Service('change_arm_laser_state', ChangeDeviceState,
                      bridge.handle_change_arm_laser_state)
        rospy.Service('change_auton_led_state', ChangeAutonLEDState,
                      bridge.handle_change_auton_led_state)
        rospy.Service('change_heater_auto_shut_off_state',
                      ChangeDeviceState,
                      bridge.handle_change_heater_auto_shut_off_state)
        rospy.Service('change_heater_state', ChangeHeaterState,
                      bridge.handle_change_heater_state)
        rospy.Service('change_servo_angles', ChangeServoAngles,
                      bridge.handle_change_servo_angles)
        rospy.Service('change_uv_led_carousel_state', ChangeDeviceState,
                      bridge.handle_change_uv_led_carousel_state)
        rospy.Service('change_uv_led_end_effector_state',
                      ChangeDeviceState,
                      bridge.handle_change_uv_led_end_effector_state)
        rospy.Service('change_white_led_state', ChangeDeviceState,
                      bridge.handle_change_white_led_state)

        while not rospy.is_shutdown():
            bridge.receive()


if __name__ == "__main__":
    main()
