#!/usr/bin/env python3

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

        self.baudrate = rospy.get_param("/science_serial/baudrate")
        # Mapping of device color to number
        self.led_map = rospy.get_param("/led_map")
        self.max_error_count = rospy.get_param("/science_info/max_error_count")
        # Mapping of onboard devices to MOSFET devices
        self.mosfet_dev_map: dict[str, int] = \
            rospy.get_param("/mosfet_device_map")
        self.nmea_handle_mapper = {
            "AUTOSHUTOFF": self.heater_auto_shut_off_handler,
            "HEATER": self.heater_state_handler,
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler
        }
        self.nmea_message_mapper = {
            "AUTOSHUTOFF": Enable(),
            "HEATER": Heater(),
            "SPECTRAL": Spectral(),
            "THERMISTOR": Thermistor(),
            "TRIAD": Spectral()
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
        self.sleep = rospy.get_param("/science_info/sleep")
        self.timeout = rospy.get_param("/science_serial/timeout")
        self.uart_transmit_msg_len = \
            rospy.get_param("/science_info/uart_transmit_msg_len")
        self.uart_lock = threading.Lock()

    def __enter__(self) -> None:
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            port=rospy.get_param("/science_serial/port"),
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=self.timeout
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()

    def add_padding(self, tx_msg: str) -> str:
        """Used to add padding since UART messages must be of certain length"""
        while len(tx_msg) < self.uart_transmit_msg_len:
            tx_msg += ","
        return tx_msg

    def auton_led_transmit(self, color: str) -> bool:
        """Sends an auton LED command message via UART"""
        try:
            requested_state = self.led_map[color.lower()]
        except KeyError:
            # Done if an invalid color is sent
            return False

        msg = f"$LED,{requested_state.value}"
        success = self.send_msg(msg)
        return success

    def format_mosfet_msg(device: int, enable: bool) -> str:
        """Formats a MOSFET message"""
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
        # tx_msg format: <"$AUTOSHUTOFF,device,enable">
        arr = tx_msg.split(",")
        enable = bool(int(arr[1]))
        ros_msg.enable = enable

    def heater_auto_shut_off_transmit(self, enable: bool) -> bool:
        """Transmits a heater auto shut off command over uart"""
        tx_msg = f"$AUTOSHUTOFF,{enable}"
        success = self.send_msg(tx_msg)
        return success

    def heater_state_handler(
            self, tx_msg: str, ros_msg: Heater) -> None:
        """Handles a received heater state message"""
        # tx_msg format: <"$HEATER,device,enable">
        arr = tx_msg.split(",")
        ros_msg.device = int(arr[1])
        ros_msg.enable = bool(int(arr[2]))

    def heater_transmit(self, device: int, enable: bool) -> bool:
        """Sends a heater state command message via UART"""
        heater_device_string = "heater_" + str(device)  # e.g. heater_0
        translated_device = self.mosfet_dev_map[heater_device_string]
        tx_msg = self.format_mosfet_msg(translated_device, int(enable))
        success = self.send_msg(tx_msg)
        return success

    def read_msg(self) -> str:
        """Used to read a message on the UART receive line"""
        error_counter = 0
        try:
            self.uart_lock.acquire()
            msg = self.ser.readline()
            self.uart_lock.release()
            return str(msg)
        except serial.SerialException as exc:
            if self.uart_lock.locked():
                self.uart_lock.release()
            print("Errored")
            if error_counter < self.max_error_count:
                error_counter += 1
                print(exc)
            else:
                raise exc

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

    def send_mosfet_msg(self, device_name: str, enable: bool) -> bool:
        """Transmits a MOSFET device state command message"""
        translated_device = self.mosfet_dev_map[device_name]
        tx_msg = self.format_mosfet_msg(translated_device, int(enable))
        success = self.send_msg(tx_msg)
        return success

    def send_msg(self, tx_msg: str) -> bool:
        """Transmits a string over UART of proper length"""
        try:
            tx_msg = self.add_padding(tx_msg)
            if len(tx_msg) > self.uart_transmit_msg_len:
                tx_msg = tx_msg[:self.uart_transmit_msg_len]
            print(tx_msg)
            self.uart_lock.acquire()
            self.ser.close()
            self.ser.open()
            self.ser.write(bytes(tx_msg, encoding='utf-8'))
            self.uart_lock.release()
            return True
        except serial.SerialException as exc:
            if self.uart_lock.locked():
                self.uart_lock.release()
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
        ros_msg_variables = [None] * 18
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
        ros_msg_variables = [None] * 18

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


def main():
    """Main function"""
    rospy.init_node("science_board")

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
